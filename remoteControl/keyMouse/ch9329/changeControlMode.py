import paho.mqtt.client as mqtt
import json
import sys
import time

# MQTT Configuration
MQTT_CONFIG_PATH = "mqtt_config.json"
CONTROL_MODE_TOPIC = "control_mode"

def load_config():
    """加载MQTT配置"""
    try:
        with open(MQTT_CONFIG_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"Failed to load config: {str(e)}")
        return None

def publish_mode(mode):
    """发布控制模式到MQTT"""
    config = load_config()
    if not config:
        print("MQTT configuration not available. Exiting.")
        return
        
    client = mqtt.Client()
    
    # 设置TLS和认证
    client.tls_set()
    client.username_pw_set(config["username"], config["password"])
    
    try:
        client.connect(config["broker"], config["port"], 60)
        print(f"Connected to MQTT broker at {config['broker']}:{config['port']}")
        
        # 发布消息
        result = client.publish(CONTROL_MODE_TOPIC, mode, qos=1)
        
        # 等待发布完成
        result.wait_for_publish()
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Successfully published '{mode}' to {CONTROL_MODE_TOPIC}")
        else:
            print(f"Failed to publish message. Error code: {result.rc}")
            
    except Exception as e:
        print(f"Connection error: {str(e)}")
    finally:
        client.disconnect()

if __name__ == "__main__":
    
    # mode ='local'
    mode ='remote'

    publish_mode(mode)