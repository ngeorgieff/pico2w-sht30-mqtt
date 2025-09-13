# Firmware and system information check
import sys
import gc

print("=== SYSTEM INFORMATION ===")
print(f"MicroPython Version: {sys.version}")
print(f"Implementation: {sys.implementation}")
print(f"Platform: {sys.platform}")

print("\n=== MEMORY INFORMATION ===")
print(f"Free memory: {gc.mem_free()} bytes")
print(f"Allocated memory: {gc.mem_alloc()} bytes")

print("\n=== MODULE AVAILABILITY CHECK ===")

# Test core modules
core_modules = [
    'micropython', 'time', 'struct', 'machine', 'gc', 'ujson'
]

for module in core_modules:
    try:
        __import__(module)
        print(f"✓ {module}")
    except ImportError:
        print(f"✗ {module} - NOT AVAILABLE")

# Test network-related modules
print("\n=== NETWORK MODULE CHECK ===")
network_modules = [
    'network', 'usocket', 'socket', 'ussl', 'ssl'
]

for module in network_modules:
    try:
        __import__(module)
        print(f"✓ {module}")
    except ImportError:
        print(f"✗ {module} - NOT AVAILABLE")

print("\n=== HELP INFO ===")
try:
    help('modules')
except:
    print("help('modules') not available")

print("\nDone!")