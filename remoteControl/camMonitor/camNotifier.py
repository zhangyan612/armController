import os
import json
import time
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import paho

TOPIC = "alert"
CHANGE_THRESHOLD = 200

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def setup_mqtt(cfg):
    # MQTT Connection callback handlers
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker successfully")
        else:
            print(f"Connection failed with error code {rc}")
    
    def on_disconnect(client, userdata, rc):
        print(f"Disconnected from broker (rc={rc})")
        if rc != 0:
            print("Unexpected disconnection, attempting to reconnect...")
            client.reconnect()
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.connect(cfg["broker"], cfg["port"], 60)
    
    # Start network loop in background thread
    client.loop_start()
    return client

def initialize_camera(camera_index=0):
    """Initialize camera and return camera object"""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_index}")
        # Try alternative camera indices
        for i in range(1, 5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Using camera index {i}")
                break
        else:
            raise Exception("No camera found")
    
    # Set camera properties for better performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    return cap

def get_camera_bottom_section(cap, height=60, right_exclude_width=150):
    """Capture a frame from camera and crop to bottom section (similar to taskbar area)"""
    ret, frame = cap.read()
    if not ret:
        raise Exception("Failed to capture frame from camera")
    
    # Get frame dimensions
    frame_height, frame_width, _ = frame.shape
    
    # Crop to bottom section, excluding right portion (like original taskbar logic)
    cropped = frame[frame_height - height : frame_height, 0 : frame_width - right_exclude_width]
    
    return cropped

def compare_images(img1, img2, threshold=30):
    """Compare two images and return the number of changed pixels"""
    diff = cv2.absdiff(img1, img2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    change_count = cv2.countNonZero(thresh)
    return change_count

def main(save_images=False, camera_index=0):
    cfg = load_config()
    client = setup_mqtt(cfg)

    if save_images:
        os.makedirs("camera_captures", exist_ok=True)

    # Initialize camera
    try:
        cap = initialize_camera(camera_index)
        print(f"Camera initialized successfully")
    except Exception as e:
        print(f"Camera initialization failed: {e}")
        return

    print("Monitoring camera bottom section every 60s...")

    # Get initial frame (bottom section only)
    try:
        prev_image = get_camera_bottom_section(cap)
        print("Initial bottom section frame captured")
        
        # Always save the first captured image
        initial_timestamp = time.strftime("%Y%m%d_%H%M%S")
        initial_filename = f"camera_captures/initial_frame_{initial_timestamp}.png"
        cv2.imwrite(initial_filename, prev_image)
        print(f"Initial frame saved as: {initial_filename}")
        
        # Publish initial alert with connection check
        if client.is_connected():
            client.publish(TOPIC, "ALERT")
            print("Initial alert published")
        else:
            print("MQTT not connected, skipping initial publish")
        time.sleep(1)

    except Exception as e:
        print(f"Failed to capture initial frame: {e}")
        cap.release()
        client.loop_stop()
        return

    try:
        while True:
            time.sleep(60)
            
            try:
                curr_image = get_camera_bottom_section(cap)
                timestamp = time.strftime("%Y%m%d_%H%M%S")

                change_count = compare_images(prev_image, curr_image)

                if change_count > CHANGE_THRESHOLD:
                    print(f"[{timestamp}] Motion detected! Pixel diff count: {change_count}")
                    
                    # Ensure connection is active before publishing
                    if not client.is_connected():
                        print("Reconnecting to MQTT broker...")
                        client.reconnect()
                        
                    result = client.publish(TOPIC, "ALERT")
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        print("Alert published successfully")
                    else:
                        print(f"Publish failed with error code: {result.rc}")
                    
                    if save_images:
                        filename = f"camera_captures/camera_{timestamp}.png"
                        cv2.imwrite(filename, curr_image)
                else:
                    print(f"[{timestamp}] No significant motion. Pixel diff count: {change_count}")

                prev_image = curr_image
                
            except Exception as e:
                print(f"Error capturing frame: {e}")
                break
                
    except KeyboardInterrupt:
        print("Monitoring stopped by user")
    finally:
        cap.release()
        client.loop_stop()
        print("Camera released and MQTT stopped")

def test_camera(camera_index=0):
    """Test function to verify camera is working and show bottom section"""
    try:
        cap = initialize_camera(camera_index)
        print("Camera test started. Press 'q' to quit.")
        print("Showing full frame and bottom section side by side")
        
        while True:
            # Get full frame
            ret, full_frame = cap.read()
            if not ret:
                break
                
            # Get bottom section
            bottom_section = get_camera_bottom_section(cap)
            
            # Display both for comparison
            cv2.imshow('Full Camera Feed', full_frame)
            cv2.imshow('Bottom Section (Monitored Area)', bottom_section)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except Exception as e:
        print(f"Camera test failed: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Uncomment the line below to test your camera first
    # test_camera()
    
    # Run the main monitoring program
    main(save_images=True, camera_index=0)