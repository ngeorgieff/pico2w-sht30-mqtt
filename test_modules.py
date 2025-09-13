# Simple test to verify all required modules are available after firmware flash
print("Testing MicroPython modules...")

# Test basic modules
modules = [
    'micropython',
    'time', 
    'struct',
    'machine',
    'gc',
    'ujson'
]

for module in modules:
    try:
        __import__(module)
        print(f"✓ {module} - OK")
    except ImportError as e:
        print(f"✗ {module} - FAILED: {e}")

# Test network module specifically
print("\nTesting network module...")
try:
    import network
    print("✓ network module imported successfully")
    
    # Test WLAN interface creation
    wlan = network.WLAN(network.STA_IF)
    print("✓ WLAN interface created")
    
    # Get MAC address to verify hardware
    mac = wlan.config('mac')
    mac_str = ':'.join(['%02x' % b for b in mac])
    print(f"✓ MAC address: {mac_str}")
    
except ImportError as e:
    print(f"✗ network module failed: {e}")
    print("You may need Raspberry Pi Pico W firmware")
except Exception as e:
    print(f"✗ network test failed: {e}")

# Test socket modules
print("\nTesting socket modules...")
try:
    import usocket as socket
    print("✓ usocket available (MicroPython)")
except ImportError:
    try:
        import socket
        print("✓ socket available (standard)")
    except ImportError:
        print("✗ No socket module available")

# Test I2C for SHT30
print("\nTesting I2C...")
try:
    from machine import I2C, Pin
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
    print("✓ I2C interface created on pins 0 (SDA), 1 (SCL)")
    
    # Scan for devices
    devices = i2c.scan()
    if devices:
        print(f"✓ I2C devices found: {[hex(addr) for addr in devices]}")
        if 0x44 in devices:
            print("✓ SHT30 sensor detected at address 0x44")
        else:
            print("⚠ SHT30 not found at 0x44, check wiring")
    else:
        print("⚠ No I2C devices found, check wiring")
        
except Exception as e:
    print(f"✗ I2C test failed: {e}")

print("\nModule test complete!")