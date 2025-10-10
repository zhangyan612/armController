import time
import json
import paho.mqtt.client as mqtt
import paho
import os 
import threading

TOPIC = "joint_state"
data_length = 12

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def decode_bitpack(payload, count):
    bitstream = int.from_bytes(payload, 'big')
    result = []
    for _ in range(count):
        result.insert(0, bitstream & ((1 << 14) - 1))
        bitstream >>= 14
    return result

class MQTTReceiver:
    def __init__(self):
        self.client = None
        self.is_connected = False
        self.message_callback = None
        self.thread = None
        self.running = False
        
    def set_message_callback(self, callback):
        """Set callback function for incoming messages"""
        self.message_callback = callback
        
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Receiver connected to MQTT broker")
            self.is_connected = True
            client.subscribe(TOPIC)
        else:
            print(f"Receiver connection failed with error code {rc}")

    def on_disconnect(self, client, userdata, rc):
        print(f"Receiver disconnected (rc={rc})")
        self.is_connected = False
        if rc != 0:
            print("Reconnecting receiver...")
            try:
                client.reconnect()
            except:
                pass

    def on_message(self, client, userdata, msg):
        try:
            # Parse the raw list of potentiometer readings
            pot_readings = decode_bitpack(msg.payload, data_length)
            print(f"Received potentiometer readings: {pot_readings}")
            
            # Call the message callback if set
            if self.message_callback:
                self.message_callback(pot_readings)
                
        except Exception as e:
            print(f"Error processing message: {str(e)}")

    def start(self):
        """Start the MQTT client in a separate thread"""
        if self.running:
            print("MQTT receiver is already running")
            return
            
        cfg = load_config()
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        
        self.client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(cfg["username"], cfg["password"])
        
        try:
            self.client.connect(cfg["broker"], cfg["port"], 60)
            self.running = True
            
            # Start network loop in a separate thread
            self.thread = threading.Thread(target=self._network_loop)
            self.thread.daemon = True
            self.thread.start()
            
            print(f"Subscribed to topic: {TOPIC}")
            print("Waiting for potentiometer readings...")
            
        except Exception as e:
            print(f"Failed to start MQTT receiver: {e}")
            self.running = False
            
    def _network_loop(self):
        """Internal method to run the network loop"""
        try:
            self.client.loop_forever()  # This runs forever until stopped
        except Exception as e:
            print(f"MQTT network loop error: {e}")
        finally:
            self.running = False
            
    def stop(self):
        """Stop the MQTT client"""
        if self.client and self.running:
            self.running = False
            self.client.disconnect()
            self.client.loop_stop()
            print("MQTT receiver stopped")


def main():
    """Standalone test function"""
    receiver = MQTTReceiver()
    
    # Test callback
    def test_callback(pot_readings):
        print(f"Test callback received: {pot_readings}")
        
    receiver.set_message_callback(test_callback)
    receiver.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        receiver.stop()

if __name__ == "__main__":
    main()