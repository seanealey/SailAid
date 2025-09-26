import streamlit as st
import pydeck as pdk
import pandas as pd
import time
import random

st.title("Smooth Live GPS Tracker (pydeck)")



# Simulated GPS updates (10 boats, 20 timesteps)
boats = [f"Boat {i}" for i in range(1, 11)]

# Starting positions (clustered around Sydney Harbour)
start_positions = {
    boat: (-33.87 + random.uniform(-0.002, 0.002),
           151.21 + random.uniform(-0.002, 0.002))
    for boat in boats
}

updates = []
for t in range(20):  # 20 updates
    update = {}
    for boat, (lat, lon) in start_positions.items():
        # Small random drift east/north
        lat += random.uniform(-0.0005, 0.0005)
        lon += random.uniform(0.0002, 0.0007)
        update[boat] = (lat, lon)
        start_positions[boat] = (lat, lon)  # save new position
    updates.append(update)

# Now you have `updates` = list of 20 dicts, each with 10 boats


map_slot = st.empty()

for update in updates:
    df = pd.DataFrame(
        [{"name": name, "lat": coords[0], "lon": coords[1]} for name, coords in update.items()]
    )

    layer = pdk.Layer(
        "ScatterplotLayer",
        data=df,
        get_position='[lon, lat]',
        get_fill_color='[200, 30, 0, 160]',
        get_radius=40,   # bigger = easier to see
        pickable=True,
    )

    view_state = pdk.ViewState(
        latitude=df["lat"].mean(),
        longitude=df["lon"].mean(),
        zoom=14,
        pitch=0,
    )

    r = pdk.Deck(layers=[layer], initial_view_state=view_state, tooltip={"text": "{name}"})

    map_slot.pydeck_chart(r)
    time.sleep(2)
