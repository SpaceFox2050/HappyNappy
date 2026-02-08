import json
from datetime import datetime
import numpy as np
import heartpy as hp
from fastapi import FastAPI
from fastapi_mqtt import FastMQTT, MQTTConfig

# --- FastAPI Setup ---
app = FastAPI(title="Sleep Tracker - Clean Start")

# --- MQTT Configuration ---
mqtt_config = MQTTConfig(
    host="localhost",
    port=1883,
    keepalive=60
)

mqtt = FastMQTT(config=mqtt_config)
mqtt.init_app(app)

# --- Global State ---
mqtt_status = {
    "receiving_packages": False,
    "total_packages": 0,
    "last_updated": None
}


@mqtt.on_connect()
def on_connect(client, flags, rc, properties):
    """Called when MQTT broker connects"""
    print("="*60)
    print("✅ Connected to MQTT Broker!")
    print("📡 Subscribed to: sensors/max30102/data")
    print("="*60 + "\n")
    mqtt.client.subscribe("sensors/max30102/data")


@mqtt.on_message()
async def on_message(client, topic, payload, qos, properties):
    """Called every time ESP32 sends data."""
    global mqtt_status
    
    try:
        # Step 1: Parse the JSON from ESP32
        data = json.loads(payload.decode())
        
        # Update status
        mqtt_status["receiving_packages"] = True
        mqtt_status["total_packages"] += 1
        mqtt_status["last_updated"] = datetime.now().isoformat()
        
        # Step 2: Extract IR array and sample rate
        ir_data = data.get('ir')
        sample_rate = data.get('sample_rate')
        
        # Step 3: Check if we have an array (not just a single value)
        if isinstance(ir_data, list) and len(ir_data) > 0:
            print(f"📩 Package #{mqtt_status['total_packages']} received at {datetime.now().strftime('%H:%M:%S')}")
            print(f"   ✅ Extracted {len(ir_data)} IR samples @ {sample_rate}Hz")
            print(f"   📊 IR range: {min(ir_data)} - {max(ir_data)}")
            
            # Step 4: Check signal quality before processing
            avg_ir = sum(ir_data) / len(ir_data)
            if avg_ir < 30000:
                print(f"   ⚠️ Poor contact (avg IR: {avg_ir:.0f}) - skipping analysis")
            else:
                try:
                    # Step 5: Apply bandpass filter to remove noise
                    filtered = hp.filter_signal(ir_data, cutoff=[0.7, 3.5], 
                                               sample_rate=sample_rate,
                                               order=3, filtertype='bandpass')
                    
                    # Step 6: Calculate BPM with enhanced accuracy
                    wd, m = hp.process(filtered, sample_rate=sample_rate,
                                      high_precision=True,
                                      clean_rr=True)
                    print(f"   💓 Calculated BPM: {m['bpm']:.2f}")
                except Exception as hp_error:
                    print(f"   ⚠️ HeartPy failed: {hp_error}")
            
        else:
            print(f"📩 Package #{mqtt_status['total_packages']} received | IR: {ir_data}")

    except Exception as e:
        print(f"❌ Error: {e}")


# --- API Endpoint ---

@app.get("/")
async def status():
    """Single endpoint showing MQTT package receipt status"""
    return {
        "mqtt_broker": "Connected" if mqtt_status["receiving_packages"] or mqtt_status["total_packages"] > 0 else "Waiting for data...",
        "receiving_packages": mqtt_status["receiving_packages"],
        "total_packages": mqtt_status["total_packages"],
        "last_updated": mqtt_status["last_updated"]
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
