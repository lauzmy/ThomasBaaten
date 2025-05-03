import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Load data
df = pd.read_csv('data/sensor_data_2025-05-02_10-28-42.csv')

# Map plot (color by speed) - updated to scatter_map
fig_map = px.scatter_map(
    df, lat="lat", lon="lon", color="speed", size="speed",
    hover_data=["time", "temperature", "heading", "rssi", "snr"],
    map_style="open-street-map", zoom=10, title="Path on Map"
)
fig_map.show()

# Time series plots
fig_ts = make_subplots(rows=5, cols=1, shared_xaxes=True, subplot_titles=[
    "Heading", "RSSI", "SNR", "Temperature", "Speed"
])

fig_ts.add_trace(go.Scatter(x=df['time'], y=df['heading'], name='Heading'), row=1, col=1)
fig_ts.add_trace(go.Scatter(x=df['time'], y=df['rssi'], name='RSSI'), row=2, col=1)
fig_ts.add_trace(go.Scatter(x=df['time'], y=df['snr'], name='SNR'), row=3, col=1)
fig_ts.add_trace(go.Scatter(x=df['time'], y=df['temperature'], name='Temperature'), row=4, col=1)
fig_ts.add_trace(go.Scatter(x=df['time'], y=df['speed'], name='Speed'), row=5, col=1)

fig_ts.update_layout(height=1000, title_text="Time Series Data")
fig_ts.show()