import sys
import json
import numpy as np
import paho.mqtt.client as paho
import heartpy as hp

# Buffer to collect sensor readings
ir_buffer = []
red_buffer = []
buffer_size = 100  # Collect 100 samples before processing
sample_rate = 25  # Hz (assuming 500ms delay = 2 samples/sec, adjust as needed)

def process_heart_rate(signal_data):
    """Process PPG signal using HeartPy to calculate BPM"""
    try:
        # Analyze the signal
        working_data, measures = hp.process(signal_data, sample_rate=sample_rate)
        
        print("\n=== Heart Rate Analysis ===")
        print(f"BPM: {measures['bpm']:.1f}")
        print(f"IBI (Inter-beat interval): {measures['ibi']:.1f} ms")
        print(f"SDNN (Std dev of NN intervals): {measures['sdnn']:.1f} ms")
        print(f"RMSSD: {measures['rmssd']:.1f} ms")
        print("===========================\n")
        
        return measures
    except Exception as e:
        print(f"Error processing heart rate: {e}")
        return None

def message_handling(client, userdata, msg):
    """Handle incoming MQTT messages"""
    global ir_buffer, red_buffer
    
    try:
        # Parse JSON payload
        data = json.loads(msg.payload.decode())
        ir_value = data.get('ir', 0)
        red_value = data.get('red', 0)
        finger_detected = data.get('finger', False)
        
        print(f"IR: {ir_value}, Red: {red_value}, Finger: {finger_detected}")
        
        # Only collect data when finger is detected
        if finger_detected:
            ir_buffer.append(ir_value)
            red_buffer.append(red_value)
            
            # Process when buffer is full
            if len(ir_buffer) >= buffer_size:
                print(f"\nProcessing {buffer_size} samples...")
                
                # Use IR signal for heart rate calculation (typically more reliable)
                signal = np.array(ir_buffer)
                process_heart_rate(signal)
                
                # Clear buffers
                ir_buffer = []
                red_buffer = []
        else:
            # Reset buffers if finger is removed
            if ir_buffer:
                print("Finger removed - clearing buffer")
                ir_buffer = []
                red_buffer = []
                
    except json.JSONDecodeError:
        print(f"Failed to parse JSON: {msg.payload.decode()}")
    except Exception as e:
        print(f"Error handling message: {e}")

def main():
    client = paho.Client(callback_api_version=paho.CallbackAPIVersion.VERSION2)
    client.on_message = message_handling

    if client.connect("localhost", 1883, 60) != 0:
        print("Couldn't connect to the MQTT broker")
        sys.exit(1)

    print("Connected to MQTT broker")
    client.subscribe("sensors/max30102/data")
    print("Subscribed to sensors/max30102/data")
    print(f"Collecting {buffer_size} samples before processing...")

    try:
        print("Press CTRL+C to exit...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nCaught keyboard interrupt, exiting...")
    finally:
        print("Disconnecting from the MQTT broker")
        client.disconnect()

if __name__ == "__main__":
    main()
