# Minimal WiFi connection test
# Update these with your actual WiFi credentials
WIFI_SSID = "IoT"  # Replace with your WiFi network name
WIFI_PASSWORD = "password"  # Replace with your WiFi password

import network
import time

def test_wifi():
    print("Testing WiFi connection...")
    
    # Create WLAN interface
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # Disable power saving for more reliable connection
    try:
        wlan.config(pm=0xa11140)
    except:
        pass
    
    print(f"Connecting to {WIFI_SSID}...")
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    
    # Wait for connection
    timeout = 20  # 20 seconds timeout
    start_time = time.time()
    
    while not wlan.isconnected() and (time.time() - start_time) < timeout:
        print(".", end="")
        time.sleep(1)
    
    print()  # New line
    
    if wlan.isconnected():
        status = wlan.ifconfig()
        print(f"✓ Connected successfully!")
        print(f"  IP Address: {status[0]}")
        print(f"  Subnet Mask: {status[1]}")
        print(f"  Gateway: {status[2]}")
        print(f"  DNS: {status[3]}")
        return True
    else:
        print("✗ Failed to connect to WiFi")
        print(f"  Status: {wlan.status()}")
        return False

if __name__ == "__main__":
    test_wifi()