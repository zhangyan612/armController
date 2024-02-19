import requests

def connect_robot():
    # Define the API endpoints
    ports_url = "http://localhost:8080/api/ports"
    connect_url = "http://localhost:8080/api/connect"
    
    # http://localhost:8080/api/connect?serialPort=COM8&baudRateSelection=0

    # Step 1: Get the available ports
    try:
        response = requests.get(ports_url)
        if response.status_code == 200:
            available_ports = response.json()
        else:
            print(f"Failed to get available ports. Status code: {response.status_code}")
            exit(1)
    except requests.exceptions.RequestException as e:
        print(f"Error making a request to {ports_url}: {e}")
        exit(1)

    # # Step 2: Find the port with the name "Prolific"
    target_port = None
    for port in available_ports:
        if port['manufacturer'] == "Prolific":
            target_port = port['path']
            break

    if target_port is None:
        print("No port with the name 'Prolific' found.")
        exit(1)

    # # Step 3: Connect to the target port
    serial_port = target_port
    baud_rate_selection = 4  # You can adjust the baud rate as needed

    connect_params = {
        "serialPort": serial_port,
        "baudRateSelection": baud_rate_selection
    }

    try:
        response = requests.post(connect_url, params=connect_params)
        if response.status_code == 200:
            print(f"Successfully connected to {target_port} (Port {serial_port})")
        else:
            print(f"Failed to connect. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error making a request to {connect_url}: {e}")
