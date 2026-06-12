"""
Quick MQTT latency tester for the Mini Pupper.

Keeps a single persistent connection open (unlike mosquitto_pub, which
reconnects every time), so you can feel how fast MQTT really is once the
connection overhead is removed.

Usage:
    pip install paho-mqtt
    python mqtt_test.py

Then type a command (e.g. "jump", "twerk", "mate", "advance") and press
Enter. The script prints how long it took for the robot to echo the
command back on minipupper/state.
"""

import time
import paho.mqtt.client as mqtt

BROKER_HOST = "192.168.1.117"
BROKER_PORT = 1883
CMD_TOPIC   = "minipupper/cmd"
STATE_TOPIC = "minipupper/state"

sent_at = {}


def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"Connected to {BROKER_HOST}:{BROKER_PORT} (rc={reason_code})")
    client.subscribe(STATE_TOPIC)


def on_message(client, userdata, msg):
    cmd = msg.payload.decode()
    t0 = sent_at.pop(cmd, None)
    if t0 is not None:
        dt_ms = (time.time() - t0) * 1000
        print(f"  <- echoed '{cmd}' on {STATE_TOPIC} after {dt_ms:.1f} ms")
    else:
        print(f"  <- {msg.topic}: {cmd}")


client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
client.loop_start()

print("Connected once. Type commands (jump, twerk, mate, advance, ...). Ctrl+C to quit.")
try:
    while True:
        cmd = input("> ").strip().lower()
        if not cmd:
            continue
        sent_at[cmd] = time.time()
        client.publish(CMD_TOPIC, cmd)
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    client.disconnect()
