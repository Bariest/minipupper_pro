"""
Quick HTTP latency tester for the Mini Pupper, for a fair comparison
against mqtt_test.py.

Uses a persistent (keep-alive) connection, same idea as mqtt_test.py
keeping one MQTT connection open, so the numbers are apples-to-apples:
both measure a full round trip from this script to the ESP32 and back.

Usage:
    pip install requests
    python http_test.py

Then type a command (e.g. "jump", "twerk", "mate", "advance") and press
Enter. The script prints how long the full HTTP request/response round
trip took.
"""

import time
import requests

ESP32_IP = "192.168.1.107"
BASE_URL = f"http://{ESP32_IP}"

# Map MQTT-style command names to the HTTP routes used by the web UI.
ROUTES = {
    "ini": "/ini", "step": "/step", "roll": "/roll", "pitch": "/pitch",
    "stretch": "/stretch", "advance": "/ad", "back": "/back",
    "left": "/left", "right": "/right", "turnl": "/turnL", "turnr": "/turnR",
    "twerk": "/twerk", "jump": "/jump", "jumpfwd": "/jumpfwd",
    "testspeed": "/testspeed", "mate": "/mate",
}

session = requests.Session()

print(f"Using persistent HTTP connection to {BASE_URL}")
print("Type commands (jump, twerk, mate, advance, ...). Ctrl+C to quit.")
try:
    while True:
        cmd = input("> ").strip().lower()
        if not cmd:
            continue
        route = ROUTES.get(cmd)
        if not route:
            print(f"  unknown command '{cmd}', known: {', '.join(ROUTES)}")
            continue

        t0 = time.time()
        resp = session.get(BASE_URL + route, timeout=5)
        dt_ms = (time.time() - t0) * 1000
        print(f"  <- {route} -> HTTP {resp.status_code} in {dt_ms:.1f} ms")
except KeyboardInterrupt:
    pass
