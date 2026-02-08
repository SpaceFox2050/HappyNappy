import sys
import json
import numpy as np
import paho.mqtt.client as paho

client = paho.Client()

if client.connect("localhost", 1883, 60) != 0:
    print("Couldn't connect to the mqtt broker")
    sys.exit(1)

# Simulate ESP32 array payload with 250 samples @ 50Hz
# Generate realistic PPG signal with heartbeat peaks at ~70 BPM
sample_rate = 50
duration = 5  # seconds
num_samples = sample_rate * duration  # 250 samples

t = np.linspace(0, duration, num_samples)
heart_rate_hz = 70 / 60  # 70 BPM

# Create realistic PPG waveform: sharp peaks + broader valleys
# Using sum of harmonics to create realistic shape
ppg_signal = (
    np.sin(2 * np.pi * heart_rate_hz * t) * 5000 +          # Main component
    np.sin(4 * np.pi * heart_rate_hz * t) * 1000 +          # Second harmonic
    np.sin(6 * np.pi * heart_rate_hz * t) * 500             # Third harmonic
)

# Add baseline and some noise
baseline = 50000
noise = np.random.normal(0, 200, num_samples)
fake_ir_samples = (baseline + ppg_signal + noise).astype(int).tolist()

payload = {
    "ir": fake_ir_samples,
    "sample_rate": sample_rate,
    "finger": True
}

json_payload = json.dumps(payload)
print(f"Publishing test message ({len(json_payload)} bytes)...")
print(f"Simulating ~70 BPM heartbeat")
print(f"Sample range: [{min(fake_ir_samples)} - {max(fake_ir_samples)}]")

client.publish("sensors/max30102/data", json_payload, 0)
print("✅ Message published!")
client.disconnect()