import os
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from dash import Dash, dcc, html, Input, Output, State

# Get the list of files in the data folder
data_folder = r'data'
data_files = [f for f in os.listdir(data_folder) if f.endswith('.csv')]

# Initialize the Dash app
app = Dash(__name__)

# Define the app layout
app.layout = html.Div([
    # Add dcc.Store components to track the last click data and selected point
    dcc.Store(id='last-ts-click-data', data=None),
    dcc.Store(id='last-map-click-data', data=None),
    dcc.Store(id='selected-point', data=None),  # Store for the selected point

    # File selection and color dropdowns
    html.Div([
        dcc.Dropdown(
            id='file-dropdown',
            options=[{'label': file, 'value': file} for file in data_files],
            placeholder="Select a data file",
            style={'width': '300px', 'margin-right': '10px'}
        ),
        dcc.Dropdown(
            id='color-dropdown',
            options=[
                {'label': 'Speed', 'value': 'speed'},
                {'label': 'Temperature', 'value': 'temperature'},
                {'label': 'Heading', 'value': 'heading'},
                {'label': 'RSSI', 'value': 'rssi'},
                {'label': 'SNR', 'value': 'snr'},
                {'label': 'Current1', 'value': 'current1'},
                {'label': 'Current2', 'value': 'current2'},
                {'label': 'Time', 'value': 'time'}
            ],
            placeholder="Select a column to color points",
            value='speed',
            style={'width': '300px'}
        )
    ], style={'display': 'flex', 'align-items': 'center', 'margin': '20px'}),

    # Map graph container
    html.Div([
        dcc.Graph(
            id='map-graph',
            config={'scrollZoom': True},
            style={'height': '50vh', 'width': '100%'},
            relayoutData=None  # Track relayout data (zoom and center)
        )
    ], style={'position': 'relative', 'height': '50vh', 'width': '100%'}),

    # Time series graph container
    html.Div([
        dcc.Graph(
            id='time-series-graph',
            style={'height': 'auto'}
        )
    ], style={'flex': '1', 'overflowY': 'auto'})
], style={
    'height': '100vh',
    'display': 'flex',
    'flexDirection': 'column',
    'margin': '0',
    'padding': '0',
    'overflow': 'hidden'
})

# Callback to load the selected file and update the graphs
@app.callback(
    [Output('map-graph', 'figure'),
     Output('time-series-graph', 'figure'),
     Output('last-ts-click-data', 'data'),
     Output('last-map-click-data', 'data'),
     Output('selected-point', 'data')],
    [Input('file-dropdown', 'value'),
     Input('color-dropdown', 'value'),
     Input('time-series-graph', 'clickData'),
     Input('map-graph', 'clickData'),
     Input('map-graph', 'relayoutData')],
    [State('last-ts-click-data', 'data'),
     State('last-map-click-data', 'data'),
     State('selected-point', 'data')]
)
def update_graphs(selected_file, selected_color, ts_click_data, map_click_data, relayout_data, last_ts_click_data, last_map_click_data, selected_point):
    if not selected_file:
        # Return empty figures if no file is selected
        return {}, {}, last_ts_click_data, last_map_click_data, selected_point

    # Load the selected file
    file_path = os.path.join(data_folder, selected_file)
    df = pd.read_csv(file_path)

    # Default zoom and center
    zoom = 12
    center = {"lat": df["lat"].mean(), "lon": df["lon"].mean()}

    # Preserve zoom and center from relayoutData
    if relayout_data:
        if "mapbox.zoom" in relayout_data:
            zoom = relayout_data["mapbox.zoom"]
        if "mapbox.center" in relayout_data:
            center = relayout_data["mapbox.center"]

    # Initialize selected_time, lat, and lon
    selected_time = None
    lat, lon = None, None

    # Determine the most recent interaction
    if last_ts_click_data != ts_click_data:
        if ts_click_data and 'points' in ts_click_data and len(ts_click_data['points']) > 0:
            clicked_point = ts_click_data['points'][0]
            selected_time = clicked_point['x']

            # Find the corresponding row in the DataFrame
            matching_row = df[df['time'] == selected_time]
            if not matching_row.empty:
                lat = matching_row.iloc[0]['lat']
                lon = matching_row.iloc[0]['lon']

            # Update the selected point
            selected_point = {"time": selected_time, "lat": lat, "lon": lon}

    elif last_map_click_data != map_click_data:
        if map_click_data and 'points' in map_click_data and len(map_click_data['points']) > 0:
            clicked_point = map_click_data['points'][0]
            lat = clicked_point['lat']
            lon = clicked_point['lon']

            # Find the corresponding row in the DataFrame
            matching_row = df[(df['lat'] == lat) & (df['lon'] == lon)]
            if not matching_row.empty:
                selected_time = matching_row.iloc[0]['time']

            # Update the selected point
            selected_point = {"time": selected_time, "lat": lat, "lon": lon}

    # Use the selected point if no new interaction occurred
    if selected_point and selected_time is None:
        selected_time = selected_point.get("time")
        lat = selected_point.get("lat")
        lon = selected_point.get("lon")

    # Create the default map figure
    fig_map = px.scatter_mapbox(
        df,
        lat="lat",
        lon="lon",
        color=selected_color,
        size="speed",
        hover_data=["time", "temperature", "heading", "rssi", "snr", "current1", "current2"],
        mapbox_style="open-street-map",
        zoom=zoom,
        center=center,
        title="Path on Map"
    )

    # Highlight the selected point on the map
    if lat is not None and lon is not None:
        fig_map.add_trace(go.Scattermapbox(
            lat=[lat],
            lon=[lon],
            mode='markers',
            marker=dict(size=15, color='red', symbol='circle'),
            name="Selected Point"
        ))

    # Create the time series figure with subplots
    fig_ts = make_subplots(
        rows=7, cols=1, shared_xaxes=True,
        subplot_titles=["Heading", "RSSI", "SNR", "Temperature", "Speed", "Current1", "Current2"]
    )

    # Add traces for each time series
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['heading'], name='Heading'), row=1, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['rssi'], name='RSSI'), row=2, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['snr'], name='SNR'), row=3, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['temperature'], name='Temperature'), row=4, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['speed'], name='Speed'), row=5, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['current1'], name='Current1'), row=6, col=1)
    fig_ts.add_trace(go.Scatter(x=df['time'], y=df['current2'], name='Current2'), row=7, col=1)

    # Highlight the selected time on the time series graph
    if selected_time is not None:
        fig_ts.add_vline(
            x=selected_time,
            line_width=2,
            line_dash="dash",
            line_color="red"
        )

    # Update layout for the time series figure
    fig_ts.update_layout(
        height=1400,
        title_text="Time Series Data"
    )

    return fig_map, fig_ts, ts_click_data, map_click_data, selected_point

# Run the app
if __name__ == '__main__':
    app.run(debug=False)