import requests
import time
import armPortConnect

def isWebServerOn(url):
    while True:
        try:
            response = requests.get(url)
            if response.status_code == 200:
                print(f"{url} is available.")
                return True
            else:
                print(f"{url} is not available (Status Code: {response.status_code}).")
        except requests.exceptions.RequestException as e:
            print(f"Error: {e}")
        
        # Wait for 5 seconds before checking again
        time.sleep(5)

if __name__ == "__main__":
    url = "http://localhost:8080"
    for _ in range(10):
        serverOn = isWebServerOn(url)
        if serverOn:
            armPortConnect.connect_robot()
            print('port connected')
            break
        else:
            print("please run startServer.bat file to start arm server. Waiting for 10 seconds...")
            time.sleep(10)

