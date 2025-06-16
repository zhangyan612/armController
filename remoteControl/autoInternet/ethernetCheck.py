import time
import subprocess
import socket

TARGET_SSID = "Your_WiFi_Name"
CHECK_INTERVAL_SECONDS = 60

def is_connected(host="8.8.8.8", port=53, timeout=3):
    """Check internet connection by trying to reach a public DNS server."""
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error:
        return False

def get_active_network_interfaces():
    """Returns a dict of connected interfaces: {name: 'Ethernet' or 'Wi-Fi'}"""
    result = subprocess.run(['netsh', 'interface', 'show', 'interface'], capture_output=True, text=True)
    lines = result.stdout.splitlines()
    interfaces = {}
    for line in lines:
        if "Connected" in line:
            parts = line.split()
            if len(parts) >= 4:
                name = parts[-1]
                type_ = parts[-2]
                interfaces[name] = type_
    return interfaces

def enable_interface(interface_name):
    subprocess.run(["netsh", "interface", "set", "interface", interface_name, "admin=enabled"],
                   capture_output=True, text=True)

def connect_to_wifi(ssid):
    print(f"📡 Trying to connect to Wi-Fi: {ssid}")
    subprocess.run(["netsh", "wlan", "connect", f"name={ssid}"], capture_output=True, text=True)

def prioritize_connection():
    interfaces = get_active_network_interfaces()
    ethernet_found = any("Ethernet" in t for t in interfaces.values())
    wifi_found = any("Wi-Fi" in t for t in interfaces.values())

    if ethernet_found:
        print("🔌 Ethernet is connected. Prioritizing it.")
        # No need to change anything if Ethernet is working
    elif wifi_found:
        print("📶 Wi-Fi is connected.")
    else:
        print("⚠️ No active connection. Attempting Wi-Fi connection...")
        connect_to_wifi(TARGET_SSID)

def main_loop():
    while True:
        print(f"\n[Checking connectivity...]")
        if is_connected():
            print("✅ Internet is connected.")
        else:
            print("❌ Internet is down.")
            prioritize_connection()
        time.sleep(CHECK_INTERVAL_SECONDS)

if __name__ == "__main__":
    main_loop()
