# Diagnostic version with detailed logging
# This will help identify where the code is failing

import micropython
import time
import struct
import network
import machine
import gc
import ujson
from machine import I2C, WDT, Pin

micropython.alloc_emergency_exception_buf(256)

# ====== USER SETTINGS ========================================================
try:
    import secrets
    WIFI_SSID  = secrets.WIFI_SSID
    WIFI_PASS  = secrets.WIFI_PASSWORD
    MQTT_HOST  = secrets.MQTT_BROKER
    MQTT_PORT  = int(getattr(secrets, "MQTT_PORT", 1883))
    MQTT_USER  = getattr(secrets, "MQTT_USER", None)
    MQTT_PASS  = getattr(secrets, "MQTT_PASSWORD", None)
    DEV_NAME   = getattr(secrets, "DEVICE_NAME", "Environmental Monitor")
    PUB_SEC    = int(getattr(secrets, "PUBLISH_INTERVAL_SEC", 300))
    print(f"Loaded config: SSID={WIFI_SSID}, MQTT={MQTT_HOST}:{MQTT_PORT}, Device={DEV_NAME}")
except (ImportError, AttributeError) as e:
    print(f"No secrets.py found: {e}")
    WIFI_SSID="ssid"; WIFI_PASS="pass"
    MQTT_HOST="10.0.0.10"; MQTT_PORT=1883; MQTT_USER=None; MQTT_PASS=None
    DEV_NAME="environmental_monitor"; PUB_SEC=300

# ====== SLUG + TOPICS ========================================================
def _slug(s):
    s = (s or "pico_sensor").lower()
    out = []
    for ch in s:
        out.append(ch if ('a' <= ch <= 'z' or '0' <= ch <= '9' or ch == '_') else '_')
    v = "".join(out).strip('_')
    return v or "pico_sensor"

DEV_ID  = _slug(DEV_NAME)
AVAIL_T = DEV_ID + "/status"
TEMP_T  = DEV_ID + "/temperature_f"
HUM_T   = DEV_ID + "/humidity"

print(f"MQTT Topics: {AVAIL_T}, {TEMP_T}, {HUM_T}")

# ====== MINIMAL MQTT (QoS0) ==================================================
try:
    import usocket as socket
except ImportError:
    import socket

class MiniMQTT:
    def __init__(self, cid, host, port=1883, user=None, pwd=None, keepalive=60):
        self.cid = cid if isinstance(cid,(bytes,bytearray)) else str(cid).encode()
        self.host, self.port = host, port
        self.user = None if user is None else (user if isinstance(user,(bytes,bytearray)) else str(user).encode())
        self.pwd  = None if pwd  is None else (pwd  if isinstance(pwd,(bytes,bytearray))  else str(pwd).encode())
        self.keep = keepalive
        self.s = None
        self.last_io = time.ticks_ms()

    def _sendall(self, b):
        mv = memoryview(b); n = 0
        while n < len(mv):
            w = self.s.write(mv[n:])
            if w is None:
                w = len(mv) - n
            if w <= 0:
                raise OSError("short write")
            n += w

    def _wlen(self, v):
        out = bytearray()
        while True:
            b = v % 128; v //= 128
            if v: b |= 0x80
            out.append(b)
            if not v: break
        self._sendall(out)

    def _rlen(self):
        mul = 1; val = 0
        while True:
            b = self.s.read(1)
            if not b: raise OSError("rlen")
            x = b[0]; val += (x & 0x7F) * mul
            if (x & 0x80) == 0: break
            mul *= 128
        return val

    def connect(self, wdt=None):
        print(f"Connecting to MQTT broker {self.host}:{self.port}...")
        if wdt: wdt.feed()
        
        try:
            ai = socket.getaddrinfo(self.host, self.port, 0, socket.SOCK_STREAM)[0][-1]
            print(f"Resolved {self.host} to {ai}")
            if wdt: wdt.feed()
            
            s = socket.socket()
            s.settimeout(10)  # Longer timeout for debugging
            s.connect(ai)
            self.s = s
            print("TCP connection established")

            flags = 0x02
            if self.user is not None:
                flags |= 0x80
                if self.pwd is not None: flags |= 0x40
            
            vh = b"\x00\x04MQTT\x04" + bytes([flags, self.keep >> 8, self.keep & 0xFF])
            pl = struct.pack("!H", len(self.cid)) + self.cid
            if self.user is not None:
                pl += struct.pack("!H", len(self.user)) + self.user
                if self.pwd is not None:
                    pl += struct.pack("!H", len(self.pwd)) + self.pwd

            self._sendall(b"\x10"); self._wlen(len(vh)+len(pl)); self._sendall(vh); self._sendall(pl)
            if wdt: wdt.feed()

            t = self.s.read(1)
            if not t or t != b"\x20": 
                raise OSError(f"Bad CONNACK: {t}")
            _ = self._rlen()
            ack = self.s.read(2)
            if not ack or ack[1] != 0x00: 
                raise OSError(f"MQTT connect failed: {ack[1] if ack else 'no response'}")
            
            self.last_io = time.ticks_ms()
            print("MQTT connected successfully!")
            
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            try: s.close()
            except: pass
            self.s = None
            raise

    def publish(self, topic, msg, retain=False):
        if not self.s: 
            raise OSError("no socket")
        
        print(f"Publishing to {topic}: {msg}")
        
        if not isinstance(topic,(bytes,bytearray)): topic = topic.encode()
        if not isinstance(msg,(bytes,bytearray)):   msg   = msg.encode()
        hdr = 0x30 | (0x01 if retain else 0x00)
        self._sendall(bytes([hdr]))
        rem = 2 + len(topic) + len(msg)
        self._wlen(rem)
        self._sendall(struct.pack("!H", len(topic))); self._sendall(topic)
        self._sendall(msg)
        self.last_io = time.ticks_ms()
        print("Published successfully")

    def close(self):
        try:
            if self.s:
                self._sendall(b"\xE0\x00")
        except: pass
        try:
            if self.s: self.s.close()
        except: pass
        self.s = None

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
        self.i2c = i2c; self.addr = addr
        self.last_t = 20.0; self.last_h = 50.0
        self._init()

    def _w(self, buf):
        try: self.i2c.writeto(self.addr, buf, True)
        except TypeError: self.i2c.writeto(self.addr, buf)

    def _init(self):
        try:
            self._w(self.CMD_SOFT_RESET);  time.sleep_ms(10)
            self._w(self.CMD_CLEAR_STATUS); time.sleep_ms(5)
            print("SHT30 initialized")
        except Exception as e:
            print(f"SHT30 init failed: {e}")

    def read(self):
        try:
            self._w(self.CMD_MEAS_HIGH_NC); time.sleep_ms(15)
            d = self.i2c.readfrom(self.addr, 6)
            if len(d) != 6: raise OSError("Bad read length")
            if _crc8(d[0:2]) != d[2] or _crc8(d[3:5]) != d[5]: raise OSError("CRC error")
            tr = (d[0] << 8) | d[1]; hr = (d[3] << 8) | d[4]
            tc = -45.0 + (175.0 * tr / 65535.0)
            rh = min(100.0, max(0.0, 100.0 * hr / 65535.0))
            self.last_t, self.last_h = tc, rh
            return tc, rh, True
        except OSError as e:
            print(f"SHT30 read failed: {e}")
            return self.last_t, self.last_h, False

# ====== WIFI HELPERS =========================================================
WLAN = network.WLAN(network.STA_IF)

def wifi_connect(ssid=WIFI_SSID, pwd=WIFI_PASS, wdt=None, total_deadline_ms=20000):
    print(f"Connecting to WiFi: {ssid}")
    start = time.ticks_ms()
    
    try:
        if WLAN.active():
            WLAN.active(False)
            time.sleep_ms(150)
    except: pass
    
    WLAN.active(True)
    
    try: WLAN.config(pm=0xa11140)  # Disable power save
    except: pass
    try: WLAN.disconnect()
    except: pass
    
    WLAN.connect(ssid, pwd)

    while True:
        if wdt: wdt.feed()
        st = WLAN.status()
        if WLAN.isconnected() and st == network.STAT_GOT_IP:
            config = WLAN.ifconfig()
            print(f"WiFi connected! IP: {config[0]}")
            return True
        if time.ticks_diff(time.ticks_ms(), start) > total_deadline_ms:
            print(f"WiFi timeout. Status: {st}")
            return False
        time.sleep_ms(100)

# ====== RUNTIME ==============================================================
# Disable watchdog for debugging
# wdt = WDT(timeout=30000)  # 30 seconds for debugging
wdt = None  # Disable watchdog completely for now

def _now_ms():
    try: return time.ticks_ms()
    except: return int(time.time() * 1000)

def run():
    print("=== Starting diagnostic version ===")
    
    # Test I2C and sensor
    print("Setting up I2C...")
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
    devices = i2c.scan()
    print(f"I2C devices: {[hex(addr) for addr in devices]}")
    
    if 0x44 not in devices:
        print("ERROR: SHT30 not found!")
        return
    
    sht = SHT30(i2c)
    
    # Test sensor reading
    temp, hum, ok = sht.read()
    if ok:
        temp_f = temp * 9/5 + 32
        print(f"Initial sensor reading: {temp:.1f}°C ({temp_f:.1f}°F), {hum:.1f}% RH")
    else:
        print("ERROR: Cannot read from sensor!")
        return

    # Connect to WiFi
    if not wifi_connect(wdt=wdt):
        print("WiFi connection failed!")
        return

    # Test MQTT connection
    print("Testing MQTT...")
    client = MiniMQTT(DEV_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=60)
    
    try:
        client.connect(wdt=wdt)
        print("MQTT connection successful!")
        
        # Test publish
        test_msg = f'{{"time": {_now_ms()}, "value": {temp_f:.1f}}}'
        client.publish(TEMP_T, test_msg, retain=True)
        client.publish(AVAIL_T, "online", retain=True)
        
        print("Test publish completed!")
        
    except Exception as e:
        print(f"MQTT test failed: {e}")
        return
    
    print("=== All tests passed! ===")
    print("Now starting monitoring loop...")
    
    # Main monitoring loop with simpler logic
    count = 0
    while count < 10:  # Only run 10 iterations for testing
        if wdt: wdt.feed()
        
        print(f"\n--- Reading {count + 1} ---")
        
        # Check WiFi
        if not WLAN.isconnected():
            print("WiFi disconnected!")
            break
            
        # Read sensor
        temp, hum, ok = sht.read()
        if ok:
            temp_f = temp * 9/5 + 32
            print(f"Sensor: {temp:.1f}°C ({temp_f:.1f}°F), {hum:.1f}% RH")
            
            # Publish data
            try:
                temp_msg = f'{{"time": {_now_ms()}, "value": {temp_f:.1f}}}'
                hum_msg = f'{{"time": {_now_ms()}, "value": {hum:.1f}}}'
                
                client.publish(TEMP_T, temp_msg)
                client.publish(HUM_T, hum_msg)
                client.publish(AVAIL_T, "online")
                
                print("Data published successfully")
                
            except Exception as e:
                print(f"Publish failed: {e}")
                break
        else:
            print("Sensor read failed")
        
        count += 1
        
        # Wait 10 seconds between readings
        for i in range(100):
            if wdt: wdt.feed()
            time.sleep_ms(100)
        
        gc.collect()
    
    print("Test completed!")
    client.close()

try:
    run()
except KeyboardInterrupt:
    print("Stopped by user")
except Exception as e:
    print(f"Unexpected error: {e}")
    import sys
    sys.print_exception(e)