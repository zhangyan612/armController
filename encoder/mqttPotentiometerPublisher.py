import time
import json
import paho.mqtt.client as mqtt
import paho
import os 
import serial

TOPIC = "joint_state"
change_threshold = 15  # Minimum change to trigger publish

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Publisher connected to MQTT broker")
    else:
        print(f"Publisher connection failed with error code {rc}")

def on_disconnect(client, userdata, rc):
    print(f"Publisher disconnected (rc={rc})")
    if rc != 0:
        print("Reconnecting publisher...")
        client.reconnect()

def hex_to_percentage(hex_data):
    percentages = []
    for i in range(0, len(hex_data), 4):
        high_byte = int(hex_data[i:i+2], 16)
        low_byte = int(hex_data[i+2:i+4], 16)
        value = (high_byte << 8) + low_byte
        percentage = value
        percentages.append(percentage)
    return percentages

def read_serial_data(ser, expected_bytes):
    data = b''
    while len(data) < expected_bytes:
        data += ser.read(expected_bytes - len(data))
    return data.hex()

def read_potentiometers():
    """
    Read actual potentiometer values from serial device.
    Returns a list of 12 potentiometer readings as numbers.
    """
    commands = [0xEE, 0xFF]
    ser = serial.Serial('COM21', 115200, timeout=1)
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            response = read_serial_data(ser, 26)  # 12 channels * 2 bytes + 2 bytes for start/end markers

            if command == 0xEE:
                percentages = hex_to_percentage(response[2:-2])  # Remove start and end markers
                ser.close()
                # Return first 12 values as a1Pos
                return percentages
    except Exception as e:
        print(f"Error reading serial data: {str(e)}")
        ser.close()
        return None

def significant_change(current, previous):
    """
    Check if any element in current list has changed by more than threshold
    compared to previous list.
    """
    if previous is None:
        return True
    
    for curr, prev in zip(current, previous):
        if abs(curr - prev) > change_threshold:
            return True
    
    return False

def publish_to_mqtt(client, readings):
    """
    Publish the readings to MQTT topic
    """
    client.publish(TOPIC, readings)
    print(f"Published to MQTT: {readings}")

# If you know the range is 0–9999, you can pack each number into 14 bits for compresson
def encode_bitpack(data):
    bitstream = 0
    for num in data:
        bitstream = (bitstream << 14) | num
    byte_length = (14 * len(data) + 7) // 8
    return bitstream.to_bytes(byte_length, 'big')

def main():
    cfg = load_config()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_start()  # Start background thread
    
    last_published = None  # Store last published values
    
    try:
        while True:
            # Read potentiometer values
            current_readings = read_potentiometers()
            
            if current_readings is not None:
                # Check if significant change occurred
                if significant_change(current_readings, last_published):
                    # Print to screen
                    print(f"Publishing encoder change: {current_readings}")
                    print(len(current_readings))
                    encoded = encode_bitpack(current_readings)
                    # Publish to MQTT
                    publish_to_mqtt(client, encoded)
                    last_published = current_readings  # Update last published values
                # else:
                #     print(f"No significant change: {current_readings}")
            else:
                print("Failed to read potentiometers")
            
            time.sleep(0.1)  # Adjust frequency as needed
            
    except KeyboardInterrupt:
        print("Publisher stopped")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()