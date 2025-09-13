# Test script to verify network module availability
# Upload this to your Pico2 W and run it to test

try:
    import network
    print("✓ Network module found!")
    
    # Test WLAN interface
    wlan = network.WLAN(network.STA_IF)
    print(f"✓ WLAN interface created: {wlan}")
    
    # Check if we can activate it
    wlan.active(True)
    print("✓ WLAN activated successfully")
    print(f"✓ MAC address: {':'.join(['%02x' % b for b in wlan.config('mac')])}")
    
except ImportError as e:
    print(f"✗ Network module not available: {e}")
    print("You need Raspberry Pi Pico W firmware with network support")
    
except Exception as e:
    print(f"✗ Error testing network: {e}")

# Test other required modules
modules_to_test = ['machine', 'time', 'struct', 'gc', 'ujson']
for module in modules_to_test:
    try:
        __import__(module)
        print(f"✓ {module} module available")
    except ImportError:
        print(f"✗ {module} module not available")