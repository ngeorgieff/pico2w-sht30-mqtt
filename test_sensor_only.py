# Simple SHT30 sensor test without network dependencies
# This will test your I2C sensor connection and basic functionality

import micropython
import time
import machine
import gc
from machine import I2C, Pin

micropython.alloc_emergency_exception_buf(256)

# ====== I2C + SHT30 ==========================================================
def _crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc

class SHT30:
    ADDR = 0x44
    CMD_SOFT_RESET   = b"\x30\xA2"
    CMD_CLEAR_STATUS = b"\x30\x41"
    CMD_MEAS_HIGH_NC = b"\x24\x00"

    def __init__(self, i2c, addr=ADDR):
        self.i2c = i2c
        self.addr = addr
        self.last_t = 20.0
        self.last_h = 50.0
        self._init()

    def _w(self, buf):
        try: 
            self.i2c.writeto(self.addr, buf, True)
        except TypeError: 
            self.i2c.writeto(self.addr, buf)

    def _init(self):
        try:
            self._w(self.CMD_SOFT_RESET)
            time.sleep_ms(10)
            self._w(self.CMD_CLEAR_STATUS)
            time.sleep_ms(5)
        except: 
            pass

    def read(self):
        try:
            self._w(self.CMD_MEAS_HIGH_NC)
            time.sleep_ms(15)
            d = self.i2c.readfrom(self.addr, 6)
            if len(d) != 6: 
                raise OSError
            if _crc8(d[0:2]) != d[2] or _crc8(d[3:5]) != d[5]: 
                raise OSError
            tr = (d[0] << 8) | d[1]
            hr = (d[3] << 8) | d[4]
            tc = -45.0 + (175.0 * tr / 65535.0)
            rh = min(100.0, max(0.0, 100.0 * hr / 65535.0))
            self.last_t, self.last_h = tc, rh
            return tc, rh, True
        except OSError:
            return self.last_t, self.last_h, False

def test_sensor():
    print("Testing SHT30 sensor...")
    
    # I2C0 on Pico 2 W: SDA=GP0, SCL=GP1
    try:
        i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
        print("✓ I2C interface created")
    except Exception as e:
        print(f"✗ I2C creation failed: {e}")
        return
    
    # Scan for devices
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(addr) for addr in devices]}")
    
    if 0x44 not in devices:
        print("✗ SHT30 not found at address 0x44")
        print("Check your wiring:")
        print("  SHT30 VCC -> Pico 3.3V")
        print("  SHT30 GND -> Pico GND")
        print("  SHT30 SDA -> Pico GP0")
        print("  SHT30 SCL -> Pico GP1")
        return
    
    print("✓ SHT30 found at address 0x44")
    
    # Test sensor readings
    sht = SHT30(i2c)
    
    for i in range(5):
        temp_c, humidity, success = sht.read()
        if success:
            temp_f = temp_c * 9/5 + 32
            print(f"Reading {i+1}: {temp_c:.1f}°C ({temp_f:.1f}°F), {humidity:.1f}% RH")
        else:
            print(f"Reading {i+1}: Failed")
        time.sleep(2)

if __name__ == "__main__":
    test_sensor()