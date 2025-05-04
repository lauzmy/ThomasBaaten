#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>

// LoRa setup
const int csPin = 53;
const int resetPin = 48;
const int irqPin = 2;

byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0x37;  // Receiver address
byte senderAddress = 0xC4; // Expected sender address

// current sensor setup
const int i1Pin = A2;
const int i2Pin = A3;
float offset1 = -2.25;
float offset2 = -3;

const int numReadings = 100;
float readingsC1[numReadings] = {0}; // the readings from the current1 sensor
float readingsC2[numReadings] = {0}; // the readings from the current1 sensor

// GPS setup
Adafruit_GPS GPS(&Serial3);
#define GPSECHO false // Set to true to see all the NMEA sentences

// IMU set up
Adafruit_BNO08x bno08x(-1);    // I2C address is 0x28 by default
long reportIntervalUs = 50000; // 50ms = 20Hz
float magX;
float magY;
float magZ;

// temperature sensor
const int TMP36_PIN = A0;

// motor controll
const int pwm1 = 6;
const int pwm2 = 5;

const int INA1 = 10;
const int INB1 = 9;
const int INA2 = 8;
const int INB2 = 7;

uint8_t m1Speed = 0;
uint8_t m2Speed = 0;
bool M1A = true;
bool M1B = true;
bool M2A = true;
bool M2B = true;

float latestHeading = 0.0;
float latestSpeed = 0.0;

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
 * @brief Updates the motor state by setting PWM and direction
 */
void updateMotorState()
{
  analogWrite(pwm1, m1Speed);
  analogWrite(pwm2, m2Speed);
  digitalWrite(INA1, M1A);
  digitalWrite(INB1, M1B);
  digitalWrite(INA2, M2A);
  digitalWrite(INB2, M2B);
}

// Returns the average of a float array
float averageReading(float arr[], int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size;
}

// Shift all values left and append new value at the end
void pushReading(float arr[], int size, float newValue) {
    for (int i = 1; i < size; i++) {
        arr[i - 1] = arr[i];
    }
    arr[size - 1] = newValue;
}

/**
 * @brief Reads the raw current from the sensor and updates the rolling buffer.
 * @param pin The analog pin to read from
 * @param offset The offset voltage to subtract
 * @param readings The array to store the readings
 * @param numReadings The number of readings to keep in the array
 */
void readRawCurrent(int pin, float offset, float readings[], int numReadings)
{
  float v = (analogRead(pin) - 512) * 5.0 / 1023.0;
  float current = (v / 0.04) - offset; // bruk sensorens 40 mV/A
  pushReading(readings, numReadings, current);
}

/**
 * @brief Reads incoming LoRa packets and updates the motor state accordingly.
 * @param packetSize The size of the incoming packet
 * @return void
 */
void onReceive(int packetSize)
{
  if (packetSize == 0)
    return;

  // Read header
  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint8_t msgId = LoRa.read();
  uint8_t payloadLength = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF)
  { // if not intended for us, ignore it
    while (LoRa.available())
      LoRa.read(); // flush rest
    return;
  }

  if (payloadLength == sizeof(MotorState))
  { // check if payload length matches expected size
    // Read payload into struct
    MotorState received;
    LoRa.readBytes((uint8_t *)&received, sizeof(received));

    m1Speed = received.M1Speed;
    m2Speed = received.M2Speed;
    // Unpack bools
    M1A = received.bools & (1 << 0);
    M1B = received.bools & (1 << 1);
    M2A = received.bools & (1 << 2);
    M2B = received.bools & (1 << 3);

    respondToSender(sender); // Respond to sender with telemetry data
  }
  else
  {
    Serial.println("Error: payload length mismatch.");
    while (LoRa.available())
      LoRa.read(); // flush
  }
}

/**
 * @returns the latest heading from the IMU
 */
float getHeading()
{
  return latestHeading;
}

/**
 * @brief gets the latest Latidude
 * @returns the latest latitude from the GPS
 */
float getLatitude()
{
  if (GPS.fix)
  {
    return GPS.latitudeDegrees;
  }
  else
  {
    return 0.0;
  }
}
/**
 * @brief gets the latest Longitude
 * @returns the latest longitude from the GPS
 */
float getLongitude()
{
  if (GPS.fix)
  {
    return GPS.longitudeDegrees;
  }
  else
  {
    return 0.0;
  }
}
/**
 * @brief gets the current temperature from the TMP36 sensor
 * @returns the current temperature in degrees celsius
 */
float getTemperature()
{
  int reading = analogRead(TMP36_PIN);
  float voltage = reading * 5.0 / 1023.0;       // Convert ADC value to voltage
  float temperatureC = (voltage - 0.5) * 100.0; // TMP36 formula
  return temperatureC;
}

/**
 * @brief gets the current speed from the GPS
 * @returns the current speed in m/s
 */
float getSpeed()
{
  if (GPS.fix)
    return GPS.speed * 0.514444; // knots to m/s
  return latestSpeed;
}

/**
 * @brief Responds to the sender with telemetry data.
 * @param sender The address to respond to
 * @return void
 */
void respondToSender(uint8_t sender)
{
  Telemetry Data;
  Data.Latitude = getLatitude();
  Data.Longitude = getLongitude();
  Data.heading = getHeading();
  Data.current1 = -averageReading(readingsC1, numReadings);
  Data.current2 = averageReading(readingsC2, numReadings);
  Data.temperature = getTemperature();
  Data.speed = getSpeed();

  LoRa.beginPacket();
  LoRa.write(sender);                         // destination address
  LoRa.write(localAddress);                   // Sender address
  LoRa.write(msgCount++);                     // Increment message ID
  LoRa.write(sizeof(Data));                   // Payload length
  LoRa.write((uint8_t *)&Data, sizeof(Data)); // Payload
  LoRa.endPacket();
}

/**
 * @brief Polls the IMU for sensor data and updates the latest heading and speed.
 * @return void
 */
void pollIMU()
{
  sh2_SensorValue_t sensorValue;
  while (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED)
    {
      float magX = sensorValue.un.magneticField.x;
      float magY = sensorValue.un.magneticField.y;
      float heading = atan2(magY, magX) * 180.0 / M_PI;
      if (heading < 0)
        heading += 360.0;
      latestHeading = heading;
    }
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
    {
      float ax = sensorValue.un.linearAcceleration.x;
      float ay = sensorValue.un.linearAcceleration.y;
      float az = sensorValue.un.linearAcceleration.z;
      latestSpeed = sqrt(ax * ax + ay * ay + az * az);
    }
  }
}

/**
 * @brief Setup function initializes the LoRa module, IMU, GPS, and motor pins.
 */
void setup()
{

  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(TMP36_PIN, INPUT);

  // begins the serial communication
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initialize LoRa
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed.");
    while (true)
      ;
  }
  Serial.println("LoRa init succeeded.");

  // initialize IMU

  if (!bno08x.begin_I2C())
  {
    Serial.println("Feil: Kunne ikke finne BNO08x via I2C!");
    while (true)
      ;
  }
  Serial.println("BNO08x funnet!");

  // Start magnetometer-rapport
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, reportIntervalUs);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);

  // intialize GPS
  Serial3.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Get minimum data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate

  delay(1000);
  Serial3.println(PMTK_Q_RELEASE); // Request firmware version
}

/**
 * @brief Main loop function that reads GPS data, updates motor state, and handles incoming LoRa packets.
 */
void loop()
{
  // read from GPS
  while (Serial3.available())
  {
    char c = GPS.read();
    if (GPSECHO)
      Serial.write(c); // Echo to serial monitor
  }

  // If a sentence is received, check if it's a valid fix
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
    {
      return; // if parse fails, do nothing. wait for next.
    }
  }

  // reinitalize IMU if it was reset
  if (bno08x.wasReset())
  {
    Serial.println("Sensor ble resatt, starter på nytt");
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, reportIntervalUs);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
  }

  pollIMU(); // poll IMU for heading and speed

  readRawCurrent(i1Pin, offset1, readingsC1, numReadings);
  readRawCurrent(i2Pin, offset2, readingsC2, numReadings);

  onReceive(LoRa.parsePacket()); // check for incoming LoRa packets
  updateMotorState();            // update motor state based on received data
}