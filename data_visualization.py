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

app.layout = html.Div([
    dcc.Store(id='last-ts-click-data', data=None),
    dcc.Store(id='last-map-click-data', data=None),
    dcc.Store(id='selected-point', data=None),

    html.Div([
        dcc.Dropdown(
            id='file-dropdown',
            options=[{'label': file, 'value': file} for file in data_files],
            placeholder="Select a data file",
            style={'width': '300px', 'margin-right': '10px'}
        ),
        dcc.Dropdown(
            id='color-dropdown',
            options=[],
            placeholder="Select a column to color points",
            style={'width': '300px'}
        )
    ], style={'display': 'flex', 'align-items': 'center', 'margin': '20px'}),

    html.Div([
        dcc.Graph(
            id='map-graph',
            config={'scrollZoom': True},
            style={'height': '50vh', 'width': '100%'},
            relayoutData=None
        )
    ], style={'position': 'relative', 'height': '50vh', 'width': '100%'}),

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

# Dynamically update color dropdown based on file
@app.callback(
    Output('color-dropdown', 'options'),
    Output('color-dropdown', 'value'),
    Input('file-dropdown', 'value')
)
def update_color_dropdown(selected_file):
    if not selected_file:
        return [], None
    file_path = os.path.join(data_folder, selected_file)
    df = pd.read_csv(file_path)
    # Only allow columns except lat/lon for coloring
    options = [
        {'label': col.title(), 'value': col}
        for col in df.columns if col not in ['lat', 'lon']
    ]
    # Default to first numeric column if available
    numeric_cols = [col for col in df.columns if pd.api.types.is_numeric_dtype(df[col]) and col not in ['lat', 'lon']]
    default_value = numeric_cols[0] if numeric_cols else None
    return options, default_value

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
        return {}, {}, last_ts_click_data, last_map_click_data, selected_point

    file_path = os.path.join(data_folder, selected_file)
    df = pd.read_csv(file_path)

    # Check for required columns
    required = {'time', 'lat', 'lon'}
    if not required.issubset(df.columns):
        return {}, {}, last_ts_click_data, last_map_click_data, selected_point

    zoom = 12
    center = {"lat": df["lat"].mean(), "lon": df["lon"].mean()}
    if relayout_data:
        if "mapbox.zoom" in relayout_data:
            zoom = relayout_data["mapbox.zoom"]
        if "mapbox.center" in relayout_data:
            center = relayout_data["mapbox.center"]

    selected_time = None
    lat, lon = None, None

    if last_ts_click_data != ts_click_data:
        if ts_click_data and 'points' in ts_click_data and len(ts_click_data['points']) > 0:
            clicked_point = ts_click_data['points'][0]
            selected_time = clicked_point['x']
            matching_row = df[df['time'] == selected_time]
            if not matching_row.empty:
                lat = matching_row.iloc[0]['lat']
                lon = matching_row.iloc[0]['lon']
            selected_point = {"time": selected_time, "lat": lat, "lon": lon}
    elif last_map_click_data != map_click_data:
        if map_click_data and 'points' in map_click_data and len(map_click_data['points']) > 0:
            clicked_point = map_click_data['points'][0]
            lat = clicked_point['lat']
            lon = clicked_point['lon']
            matching_row = df[(df['lat'] == lat) & (df['lon'] == lon)]
            if not matching_row.empty:
                selected_time = matching_row.iloc[0]['time']
            selected_point = {"time": selected_time, "lat": lat, "lon": lon}
    if selected_point and selected_time is None:
        selected_time = selected_point.get("time")
        lat = selected_point.get("lat")
        lon = selected_point.get("lon")

    # Create the map figure
    fig_map = px.scatter_mapbox(
        df,
        lat="lat",
        lon="lon",
        color=selected_color if selected_color in df.columns else None,
        size=None,
        hover_data=[col for col in df.columns if col not in ['lat', 'lon']],
        mapbox_style="open-street-map",
        zoom=zoom,
        center=center,
        title="Path on Map"
    )
    if lat is not None and lon is not None:
        fig_map.add_trace(go.Scattermapbox(
            lat=[lat],
            lon=[lon],
            mode='markers',
            marker=dict(size=15, color='red', symbol='circle'),
            name="Selected Point"
        ))

    # Dynamically create time series subplots for all numeric columns except lat/lon/time
    time_col = 'time'
    # Only plot numeric columns except lat, lon, and time
    plot_cols = [col for col in df.columns if pd.api.types.is_numeric_dtype(df[col]) and col not in ['lat', 'lon', time_col]]
    subplot_titles = [col.title() for col in plot_cols]
    fig_ts = make_subplots(
        rows=len(plot_cols), cols=1, shared_xaxes=True,
        subplot_titles=subplot_titles
    )
    for i, col in enumerate(plot_cols):
        fig_ts.add_trace(go.Scatter(x=df[time_col], y=df[col], name=col.title()), row=i+1, col=1)

    # Highlight the selected time on the time series graph
    if selected_time is not None:
        fig_ts.add_vline(
            x=selected_time,
            line_width=2,
            line_dash="dash",
            line_color="red"
        )

    fig_ts.update_layout(
        height=200 * max(1, len(subplot_titles)),
        title_text="Time Series Data"
    )

    return fig_map, fig_ts, ts_click_data, map_click_data, selected_point

if __name__ == '__main__':
    app.run(debug=True)