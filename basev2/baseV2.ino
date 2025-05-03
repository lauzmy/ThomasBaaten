#include <SPI.h> // include libraries
#include <LoRa.h>

// lora
const int csPin = 10;   // LoRa radio chip select
const int resetPin = 9; // LoRa radio reset
const int irqPin = 8;   // hardware interrupt pin

byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xC4; // address of this device
byte destination = 0x37;  // destination to send to

// motor control
const int speed1 = 75;  // 75 / 255 = 29.4%
const int speed2 = 150; // 150 / 255 = 58.8%
const int speed3 = 255; // 255 / 255 = 100%

uint8_t m1Speed = speed1;
bool M1A = true;
bool M1B = true;

uint8_t m2Speed = speed1;
bool M2A = true;
bool M2B = true;

uint8_t bools = 0;

float heading = 0.0;       // current heading from the IMU
float targetHeading = 0.0; // target heading

struct MotorState
{
  uint8_t M1Speed; // speed of motor 1
  uint8_t M2Speed; // speed of motor 2
  uint8_t bools;   // packed bools for motor control
};

struct Telemetry
{
  float Latitude;
  float Longitude;
  float heading;
  float current1;
  float current2;
  float temperature;
  float speed;
};

/**
 * @brief Drives the motors toward a target heading.
 * @param current The current heading of the robot
 * @param target The target heading to drive toward
 */
void driveTowardTargetHeading(float current, float target)
{

  // Calculate shortest angle difference (-180 to 180)
  float diff = target - current;
  if (diff > 180)
    diff -= 360;
  if (diff < -180)
    diff += 360;

  // Threshold for "close enough" (degrees)
  const float threshold = 10.0;

  if (abs(diff) < threshold)
  {
    // Go straight
    M1A = true;
    M1B = false;
    M2A = true;
    M2B = false;
  }
  else if (diff > 0)
  {
    // Turn right
    m1Speed = speed1;
    m2Speed = speed1;

    M1A = true;
    M1B = true;
    M2A = true;
    M2B = false;
  }
  else
  {
    // Turn left
    m1Speed = speed1;
    m2Speed = speed1;

    M1A = true;
    M1B = false;
    M2A = true;
    M2B = true;
  }
}

/**
 * @brief Sends a message over LoRa with the specified payload.
 * @param payload The message to send
 */
void sendMessage(MotorState payload)
{
  LoRa.beginPacket();                               // start packet
  LoRa.write(destination);                          // add destination address
  LoRa.write(localAddress);                         // add sender address
  LoRa.write(msgCount);                             // add message ID
  LoRa.write(sizeof(payload));                      // add payload length
  LoRa.write((uint8_t *)&payload, sizeof(payload)); // add payload
  LoRa.endPacket();                                 // finish packet and send it
  msgCount++;                                       // increment message ID
}

/**
 * @brief Reads incoming LoRa packets and processes them.
 * @param packetSize The size of the incoming packet
 */
void onReceive(int packetSize)
{
  if (packetSize == 0)
    return;

  // read packet header
  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t payloadLength = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF)
    return; // if not intended for us, ignore it

  if (payloadLength == sizeof(Telemetry))
  { // check if payload length matches expected size
    // Read payload into struct
    Telemetry data;
    LoRa.readBytes((uint8_t *)&data, sizeof(data));

    // Print telemetry data
    Serial.print("lat: ");
    Serial.println(data.Latitude, 6);
    Serial.print("lon: ");
    Serial.println(data.Longitude, 6);
    Serial.print("heading: ");
    Serial.println(data.heading);
    heading = data.heading;
    Serial.print("current1: ");
    Serial.println(data.current1);
    Serial.print("current2: ");
    Serial.println(data.current2);
    Serial.print("temperature: ");
    Serial.println(data.temperature);
    Serial.print("speed: ");
    Serial.println(data.speed);
    Serial.print("rssi: ");
    Serial.println(LoRa.packetRssi());
    Serial.print("snr: ");
    Serial.println(LoRa.packetSnr());
  }
  else
  {
    Serial.println("Invalid payload length");
    while (LoRa.available())
      LoRa.read(); // flush
  }
}

/**
 * @brief Packs four boolean values into a single byte.
 @returns the packed byte
 */
uint8_t packBools(bool b1, bool b2, bool b3, bool b4)
{
  uint8_t bools = 0;
  bools |= b1 << 0;
  bools |= b2 << 1;
  bools |= b3 << 2;
  bools |= b4 << 3;
  return (bools);
};

/**
 * @brief Sets up the LoRa module, initializes serial communication.
 */
void setup()
{

  // Initialize serial
  Serial.begin(115200);

  // override the default CS, reset, and IRQ pins
  LoRa.setPins(csPin, resetPin, irqPin);

  // initialize ratio at 915 MHz
  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed.");
  }
  else
  {
    Serial.println("LoRa init succeeded.");
  }
}

/**
 * @brief Main loop that checks for incoming commands and processes them.
 */
void loop()
{
  // Check if data is available on the serial port
  if (Serial.available() > 0)
  {
    // Read the incoming data
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any extra whitespace or newline characters

    // processing recived commands

    // set target heading and starts route
    if (command.startsWith("SETHEADING:"))
    {
      int sep = command.indexOf(':');
      if (sep != -1)
      {
        String headingStr = command.substring(sep + 1);
        targetHeading = headingStr.toFloat();
        Serial.print("Set target heading to: ");
        Serial.println(targetHeading);
      }
    }
    // reset target heading and cancel route
    if (command == "CANCELROUTE")
    {
      targetHeading = 0.0; // Reset target heading to 0
      Serial.println("Route cancelled");
    }
    if (targetHeading == 0.0)
    {
      // If target heading is 0, stop the motors
      M1A = true;
      M1B = true;
      M2A = true;
      M2B = true;
    }
    else
    {
      // If target heading is not 0, drive toward the target heading
      driveTowardTargetHeading(heading, targetHeading);
    }
    // move forward
    if (command == "FORWARD")
    {
      Serial.println("Moving forward");
      M1A = true;
      M1B = false;
      M2A = true;
      M2B = false;
    }
    // move backward
    if (command == "BACKWARD")
    {
      Serial.println("Moving backward");
      m1Speed = speed1;
      m2Speed = speed1;

      M1A = false;
      M1B = true;
      M2A = false;
      M2B = true;
    }
    // turn left
    if (command == "TURNLEFT")
    {
      Serial.println("Turning left");
      m1Speed = speed1;
      m2Speed = speed1;

      M1A = true;
      M1B = false;
      M2A = true;
      M2B = true;
    }
    // turn right
    if (command == "TURNRIGHT")
    {
      Serial.println("Turning right");
      m1Speed = speed1;
      m2Speed = speed1;

      M1A = true;
      M1B = true;
      M2A = true;
      M2B = false;
    }
    // set speed to 1
    if (command == "SPEED1")
    {
      Serial.println("Setting speed to 1");
      m1Speed = speed1;
      m2Speed = speed1;
    }
    // set speed to 2
    if (command == "SPEED2")
    {
      Serial.println("Setting speed to 2");
      m1Speed = speed2;
      m2Speed = speed2;
    }
    //  set speed to 3
    if (command == "SPEED3")
    {
      Serial.println("Setting speed to 3");
      m1Speed = speed3;
      m2Speed = speed3;
    }
    // stop the motors
    if (command == "STOP")
    {
      Serial.println("Stopping motors");
      M1A = true;
      M1B = true;
      M2A = true;
      M2B = true;
    }
  }

  static uint32_t timer = millis();
  // every 100ms send motor data
  if (millis() - timer > 100)
  {
    MotorState payload;
    payload.m1Speed = M1Speed;
    payload.m2Speed = M2Speed;
    payload.bools = packBools(M1A, M1B, M2A, M2B);
    sendMessage(payload);
    timer = millis();
  }
  onReceive(LoRa.parsePacket()); // check for packet
}
