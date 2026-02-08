import json
import math
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

# --- BPM History for visualization ---
bpm_history = []  # Stores {"timestamp": ..., "bpm": ...}
start_time = None  # Track when monitoring started

# --- Wake-up Alert & Sleep Detection ---
awake = False  # DEMO MODE: Set to True to trigger alarm immediately

baseline_bpm = None  # Average BPM from first 10 readings
sleep_stage = "awake"  # "awake", "falling_asleep", "deep_sleep"
demo_triggered = False  # Track if demo alarm already sent


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
            if avg_ir < 900:
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
                    current_bpm = m['bpm']
                    print(f"   💓 Calculated BPM: {current_bpm:.2f}")
                    
                    # Store BPM with timestamp (only if valid)
                    global start_time, baseline_bpm, awake, sleep_stage, demo_triggered
                    if start_time is None:
                        start_time = datetime.now()
                    
                    # Validate BPM is a finite number (not nan, inf, -inf)
                    if math.isfinite(current_bpm):
                        elapsed_seconds = (datetime.now() - start_time).total_seconds()
                        bpm_history.append({
                            "time": elapsed_seconds,
                            "bpm": round(current_bpm, 2)
                        })
                    else:
                        print(f"   ⚠️ Invalid BPM value ({current_bpm}) - skipping")
                    
                    # === DEMO MODE: Trigger alarm on first reading ===
                    if awake and not demo_triggered:
                        print(f"\n{'='*60}")
                        print(f"🎯 DEMO MODE: Triggering wake alert!")
                        print(f"{'='*60}\n")
                        
                        demo_triggered = True
                        sleep_stage = "demo_mode"
                        
                        # Send wake alert to ESP32
                        mqtt.client.publish("esp32/wake_alert", "WAKE_UP", qos=0)
                        mqtt.client.publish("esp32/control", "STOP_SAMPLING", qos=0)
                        print("📤 Sent WAKE_UP and STOP_SAMPLING to ESP32")
                    
                    # === SLEEP STAGE DETECTION ===
                    
                    # Step 1: Establish baseline from first 60 readings
                    if baseline_bpm is None and len(bpm_history) >= 60:
                        baseline_bpm = sum(reading["bpm"] for reading in bpm_history[:60]) / 60
                        print(f"\n✅ Baseline BPM established: {baseline_bpm:.2f}")
                        print(f"   Will alert if BPM drops below {baseline_bpm * 0.75:.2f}\n")
                    
                    # Step 2: Monitor for deep sleep (20-25% drop from baseline)
                    if baseline_bpm is not None and not awake:
                        bpm_drop_percent = ((baseline_bpm - current_bpm) / baseline_bpm) * 100
                        
                        if bpm_drop_percent >= 20:  # 20% drop = entering deep sleep
                            print(f"\n{'='*60}")
                            print(f"🚨 DEEP SLEEP DETECTED! 🚨")
                            print(f"   Baseline: {baseline_bpm:.2f} BPM")
                            print(f"   Current:  {current_bpm:.2f} BPM")
                            print(f"   Drop:     {bpm_drop_percent:.1f}%")
                            print(f"{'='*60}\n")
                            
                            # Trigger wake alert
                            awake = True
                            sleep_stage = "deep_sleep"
                            
                            # Send wake alert to ESP32
                            mqtt.client.publish("esp32/wake_alert", "WAKE_UP", qos=0)
                            # Tell ESP32 to stop sending data
                            mqtt.client.publish("esp32/control", "STOP_SAMPLING", qos=0)
                            print("📤 Sent WAKE_UP and STOP_SAMPLING to ESP32")
                        
                        elif bpm_drop_percent >= 10:  # 10-20% drop = falling asleep
                            if sleep_stage != "falling_asleep":
                                sleep_stage = "falling_asleep"
                                print(f"   😴 Falling asleep... (BPM dropped {bpm_drop_percent:.1f}%)")
                    
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
        "last_updated": mqtt_status["last_updated"],
        "sleep_stage": sleep_stage,
        "baseline_bpm": baseline_bpm,
        "awake_alert_triggered": awake
    }


@app.get("/bpm_history")
async def get_bpm_history():
    """Get BPM history for visualization"""
    return {
        "data": bpm_history,
        "total_readings": len(bpm_history),
        "current_bpm": bpm_history[-1]["bpm"] if bpm_history else None
    }


@app.post("/wake_alert")
async def trigger_wake_alert():
    """MANUAL TEST ONLY - Trigger wake-up alert (automatic detection happens in on_message)"""
    global awake
    awake = True
    
    try:
        # Publish alert message to ESP32
        mqtt.client.publish("esp32/wake_alert", "WAKE_UP", qos=0)
        mqtt.client.publish("esp32/control", "STOP_SAMPLING", qos=0)
        print("🚨 Wake alert sent to ESP32 (manual trigger)!")
        return {"status": "success", "message": "Wake alert sent to ESP32"}
    except Exception as e:
        print(f"❌ Failed to send wake alert: {e}")
        return {"status": "error", "message": str(e)}


@app.get("/awake_status")
async def get_awake_status():
    """Get current awake status"""
    return {"awake": awake}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
