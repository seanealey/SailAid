# fleet_tracker_qt.py
import sys
import json
import math
import random
import statistics
from argparse import ArgumentParser

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtWebEngineWidgets import QWebEngineView
import serial

# ---- CONFIGURE YOUR SERIAL PORT ----
SERIAL_PORT = "COM8"        # e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux
BAUD_RATE = 115200
SERIAL_TIMEOUT_S = 0.05

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>Fleet Tracker</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link
  rel="stylesheet"
  href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
  integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
  crossorigin=""
/>
<style>
  html, body, #map { height: 100%; margin: 0; }
  .marker-label {
    background: rgba(255,255,255,0.85);
    padding: 2px 6px;
    border-radius: 4px;
    border: 1px solid #888;
    font-size: 12px;
    white-space: nowrap;
  }
</style>
</head>
<body>
<div id="map"></div>
<script
  src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
  integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
  crossorigin="">
</script>
<script>
  var map = L.map('map', { zoomControl: true });
  map.setView([-33.8688, 151.2093], 12);

  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 20,
    attribution: '&copy; OpenStreetMap'
  }).addTo(map);

  const markers = {};
  const labels = {};

  function upsertMarker(unitId, lat, lon, colorRGB, nameText) {
    const color = 'rgb(' + colorRGB.join(',') + ')';
    if (!markers[unitId]) {
      markers[unitId] = L.circleMarker([lat, lon], {
        radius: 6, color: color, fillColor: color, fillOpacity: 0.9, weight: 1
      }).addTo(map);
      labels[unitId] = L.marker([lat, lon], {
        icon: L.divIcon({ className: 'marker-label', html: nameText, iconAnchor: [0, 0] })
      }).addTo(map);
    } else {
      markers[unitId].setLatLng([lat, lon]).setStyle({ color: color, fillColor: color });
      labels[unitId].setLatLng([lat, lon]).setIcon(L.divIcon({
        className: 'marker-label', html: nameText, iconAnchor: [0, 0]
      }));
    }
  }

  function setMapView(lat, lon, zoom) { map.setView([lat, lon], zoom); }

  window.fleet = { upsertMarker, setMapView };
</script>
</body>
</html>
"""

class DemoDataGenerator:
    """
    Simple random-walk demo generator that emits FLEET lines:
        FLEET,<id>,<lat>,<lon>
    IDs: 101/102 boats, 201/202 buoys, 301 central node
    """
    def __init__(self, center_lat=-33.76896, center_lon=151.28620):
        random.seed(42)
        self.points = {
            101: [center_lat + 0.0008, center_lon - 0.0010],
            102: [center_lat - 0.0007, center_lon + 0.0008],
            201: [center_lat + 0.0002, center_lon + 0.0002],
            202: [center_lat - 0.0003, center_lon - 0.0001],
            301: [center_lat, center_lon],  # central node
        }

    def step(self):
        lines = []
        for uid, (lat, lon) in self.points.items():
            # Small random walk; central node drifts less
            jitter = 0.00008 if str(uid).startswith("3") else 0.00025
            bearing = random.uniform(0, 2 * math.pi)
            dlat = math.sin(bearing) * jitter
            dlon = math.cos(bearing) * jitter / max(math.cos(math.radians(lat)), 0.2)
            lat += dlat
            lon += dlon
            self.points[uid] = [lat, lon]
            lines.append(f"FLEET,{uid},{lat:.6f},{lon:.6f}")
        return lines

class FleetWindow(QMainWindow):
    def __init__(self, demo_mode=False):
        super().__init__()
        self.setWindowTitle("Fleet Tracker (Desktop)")
        self.resize(900, 700)

        self.view = QWebEngineView()
        self.view.setHtml(HTML_TEMPLATE)
        self.setCentralWidget(self.view)

        # State
        self.latest_positions = {}  # id -> (lat, lon)
        self.central_node = None    # (lat, lon)

        # Demo
        self.demo_mode = demo_mode
        self.demo = DemoDataGenerator()

        # Serial setup
        self.ser = None
        if not self.demo_mode:
            try:
                self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT_S)
                print(f"Opened serial port {SERIAL_PORT} @ {BAUD_RATE}")
            except Exception as e:
                print(f"Failed to open serial port {SERIAL_PORT}: {e}")
                print("Falling back to demo mode.")
                self.demo_mode = True

        # Timers
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # poll serial / generate demo lines
        self.timer.timeout.connect(self.tick)
        self.timer.start()

        self.view_timer = QTimer(self)
        self.view_timer.setInterval(500)  # recenter/refresh view
        self.view_timer.timeout.connect(self.update_view)
        self.view_timer.start()

    def closeEvent(self, event):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        return super().closeEvent(event)

    # ----- Common processing -----
    def process_line(self, line: str):
        # Expected: FLEET,<id>,<lat>,<lon>
        if not line.startswith("FLEET"):
            return
        parts = line.strip().split(",")
        if len(parts) != 4:
            return
        _, id_str, lat_str, lon_str = parts
        try:
            unit_id = int(id_str)
            lat = float(lat_str)
            lon = float(lon_str)
        except ValueError:
            return

        self.latest_positions[unit_id] = (lat, lon)

        sid = str(unit_id)
        if sid.startswith("1"):
            color = [0, 0, 255]
            name = f"Boat {unit_id}"
        elif sid.startswith("2"):
            color = [255, 0, 0]
            name = f"Buoy {unit_id}"
        elif sid.startswith("3"):
            color = [0, 200, 0]
            name = f"Central {unit_id}"
            if lat != 0 and lon != 0:
                self.central_node = (lat, lon)
        else:
            color = [100, 100, 100]
            name = f"Unit {unit_id}"

        js = (
            "window.fleet.upsertMarker({id}, {lat}, {lon}, {color}, {name});"
            .format(
                id=json.dumps(unit_id),
                lat=json.dumps(lat),
                lon=json.dumps(lon),
                color=json.dumps(color),
                name=json.dumps(name),
            )
        )
        self.view.page().runJavaScript(js)

    # ----- Timers -----
    def tick(self):
        if self.demo_mode:
            # Generate a batch of demo lines each tick
            for line in self.demo.step():
                self.process_line(line)
        else:
            if not self.ser:
                return
            try:
                while True:
                    line = self.ser.readline()
                    if not line:
                        break
                    self.process_line(line.decode("utf-8", errors="ignore"))
            except Exception as e:
                print("Serial/parse error:", e)

    def update_view(self):
        if not self.latest_positions:
            return
        if self.central_node:
            view_lat, view_lon = self.central_node
        else:
            lats = [v[0] for v in self.latest_positions.values()]
            lons = [v[1] for v in self.latest_positions.values()]
            try:
                view_lat = statistics.fmean(lats)
                view_lon = statistics.fmean(lons)
            except Exception:
                view_lat, view_lon = 0.0, 0.0

        js = "window.fleet.setMapView({lat}, {lon}, {zoom});".format(
            lat=json.dumps(view_lat), lon=json.dumps(view_lon), zoom=18
        )
        self.view.page().runJavaScript(js)

def main():
    ap = ArgumentParser(description="Fleet Tracker (Desktop) with optional demo mode")
    ap.add_argument("--demo", action="store_true", help="Run with simulated data instead of serial")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("Fleet Tracker (Desktop)")
    w = FleetWindow(demo_mode=args.demo)
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
