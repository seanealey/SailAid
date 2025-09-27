import streamlit as st
import pydeck as pdk
import pandas as pd
import serial
import time

st.title("Fleet Tracker")

# ---- CONFIGURE YOUR SERIAL PORT ----
# Windows: "COM3" / "COM4"
# Linux/Mac: "/dev/ttyUSB0" or "/dev/ttyAMA0"
SERIAL_PORT = "COM8"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Keep track of latest positions
latest_positions = {}
central_node = None  # stores last valid central node position

map_slot = st.empty()

while True:
    line = ser.readline().decode("utf-8").strip()
    if line.startswith("FLEET"):
        try:
            _, id_str, lat_str, lon_str = line.split(",")
            unit_id = int(id_str)
            lat, lon = float(lat_str), float(lon_str)

            # Store latest position
            latest_positions[unit_id] = (lat, lon)

            df = []
            for uid, (ulat, ulon) in latest_positions.items():
                if str(uid).startswith("1"):  # Boats
                    df.append({"name": f"Boat {uid}", "lat": ulat, "lon": ulon, "color": [0, 0, 255]})
                elif str(uid).startswith("2"):  # Buoys
                    df.append({"name": f"Buoy {uid}", "lat": ulat, "lon": ulon, "color": [255, 0, 0]})
                elif str(uid).startswith("3"):  # Central Node
                    df.append({"name": f"Central {uid}", "lat": ulat, "lon": ulon, "color": [0, 200, 0]})
                    if ulat != 0 and ulon != 0:  # Only use if valid
                        central_node = (ulat, ulon)

            df = pd.DataFrame(df)

            # Center on central node if valid, otherwise average of all points
            if central_node:
                view_lat, view_lon = central_node
            else:
                view_lat, view_lon = df["lat"].mean(), df["lon"].mean()

            # Create pydeck layer
            layer = pdk.Layer(
                "ScatterplotLayer",
                data=df,
                get_position='[lon, lat]',
                get_fill_color="color",
                get_radius=4,
                pickable=True,
            )

            view_state = pdk.ViewState(
                latitude=view_lat,
                longitude=view_lon,
                zoom=18,
                pitch=0,
            )

            r = pdk.Deck(layers=[layer], initial_view_state=view_state, tooltip={"text": "{name}"})
            map_slot.pydeck_chart(r)

        except Exception as e:
            st.write("Parse error:", e)

    time.sleep(0.2)  # avoid busy loop
