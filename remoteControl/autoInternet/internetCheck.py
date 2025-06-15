import time
import subprocess
import socket

TARGET_SSID = "Verizon_YGCW99"
CHECK_INTERVAL_SECONDS = 60

def is_connected(host="8.8.8.8", port=53, timeout=3):
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error:
        return False

def connect_to_wifi(ssid):
    print(f"Attempting to connect to Wi-Fi: {ssid}")
    try:
        result = subprocess.run(["netsh", "wlan", "connect", f"name={ssid}"], capture_output=True, text=True)
        print("Connection output:", result.stdout)
        if result.returncode != 0:
            print("Failed to connect:", result.stderr)
    except Exception as e:
        print("Error during Wi-Fi connect:", e)

def main_loop():
    while True:
        if is_connected():
            print("✅ Internet connected")
        else:
            print("❌ No internet connection. Reconnecting...")
            connect_to_wifi(TARGET_SSID)
        time.sleep(CHECK_INTERVAL_SECONDS)

if __name__ == "__main__":
    main_loop()
