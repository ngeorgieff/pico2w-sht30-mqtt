# Pico 2 W + SHT30 to Home Assistant MQTT (with Discovery) - optimized & hardened
# Tested on MicroPython for RP2350 (Pico 2 W). Place as main.py
import micropython
import time
import struct
import network
import machine
import gc
import ujson
from machine import I2C
from machine import WDT
from machine import Pin

micropython.alloc_emergency_exception_buf(256)

# ====== USER SETTINGS (from secrets.py if present) ============================
try:
    import secrets
    WIFI_SSID  = secrets.WIFI_SSID
    WIFI_PASS  = secrets.WIFI_PASSWORD
    MQTT_HOST  = secrets.MQTT_BROKER
    MQTT_PORT  = int(getattr(secrets, "MQTT_PORT", 1883))
    MQTT_USER  = getattr(secrets, "MQTT_USER", None)
    MQTT_PASS  = getattr(secrets, "MQTT_PASSWORD", None)
    DEV_NAME   = getattr(secrets, "DEVICE_NAME", "Network Rack")
    PUB_SEC    = int(getattr(secrets, "PUBLISH_INTERVAL_SEC", 300))
except (ImportError, AttributeError):
    WIFI_SSID="ssid"; WIFI_PASS="pass"
    MQTT_HOST="10.0.0.10"; MQTT_PORT=1883; MQTT_USER=None; MQTT_PASS=None
    DEV_NAME="Network Rack"; PUB_SEC=300

# ====== HOME ASSISTANT MQTT DISCOVERY SETTINGS ================================
# Home Assistant MQTT Discovery prefix (standard)
HA_DISCOVERY_PREFIX = "homeassistant"

# Device identifier (used in topics and entity IDs - keep it short!)
# Entity IDs will be: sensor.network_rack_temperature, sensor.network_rack_humidity
DEVICE_ID = "network_rack"

# Home Assistant MQTT Topics
# Discovery config topics
TEMP_CONFIG_TOPIC = f"{HA_DISCOVERY_PREFIX}/sensor/{DEVICE_ID}/temperature/config"
HUM_CONFIG_TOPIC = f"{HA_DISCOVERY_PREFIX}/sensor/{DEVICE_ID}/humidity/config"

# State topics
TEMP_STATE_TOPIC = f"{HA_DISCOVERY_PREFIX}/sensor/{DEVICE_ID}/temperature/state"
HUM_STATE_TOPIC = f"{HA_DISCOVERY_PREFIX}/sensor/{DEVICE_ID}/humidity/state"

# Availability topic (shared for all sensors from this device)
AVAIL_TOPIC = f"{HA_DISCOVERY_PREFIX}/sensor/{DEVICE_ID}/availability"

# ====== MINIMAL MQTT (QoS0) ==================================================
try:
    import usocket as socket
except ImportError:
    import socket
try:
    import ussl as sslmod
except ImportError:
    sslmod = None

class MiniMQTT:
    def __init__(self, cid, host, port=1883, user=None, pwd=None, keepalive=60, use_ssl=False, ssl_params=None):
        self.cid = cid if isinstance(cid,(bytes,bytearray)) else str(cid).encode()
        self.host, self.port = host, port
        self.user = None if user is None else (user if isinstance(user,(bytes,bytearray)) else str(user).encode())
        self.pwd  = None if pwd  is None else (pwd  if isinstance(pwd,(bytes,bytearray))  else str(pwd).encode())
        self.keep = keepalive
        self.use_ssl  = use_ssl
        self.ssl_params = ssl_params or {}
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

    def _send_str(self, s):
        if not isinstance(s,(bytes,bytearray)): s = str(s).encode()
        self._sendall(struct.pack("!H", len(s))); self._sendall(s)

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
        # DNS + TCP with short, bounded timeouts
        if wdt: wdt.feed()
        
        try:
            ai = socket.getaddrinfo(self.host, self.port, 0, socket.SOCK_STREAM)[0][-1]
            if wdt: wdt.feed()
            
            s = socket.socket()
            s.settimeout(5)
            s.connect(ai)
            
            if self.use_ssl:
                if sslmod is None:
                    raise OSError("ssl not available")
                s = sslmod.wrap_socket(s, **self.ssl_params)
            self.s = s

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
            if not t or t != b"\x20": raise OSError(f"Bad CONNACK header: {t}")
            _ = self._rlen()
            ack = self.s.read(2)
            if not ack or ack[1] != 0x00: 
                error_codes = {1: "Unacceptable protocol version", 2: "Identifier rejected", 
                              3: "Server unavailable", 4: "Bad user/password", 5: "Not authorized"}
                error_msg = error_codes.get(ack[1] if ack else 0, f"Unknown error: {ack[1] if ack else 'no response'}")
                raise OSError(f"MQTT connect failed: {error_msg}")
            
            self.last_io = time.ticks_ms()
            
        except Exception as e:
            try: s.close()
            except: pass
            self.s = None
            raise

    def publish(self, topic, msg, retain=False):
        if not self.s: raise OSError("no socket")
        if not isinstance(topic,(bytes,bytearray)): topic = topic.encode()
        if not isinstance(msg,(bytes,bytearray)):   msg   = msg.encode()
        hdr = 0x30 | (0x01 if retain else 0x00)
        self._sendall(bytes([hdr]))
        rem = 2 + len(topic) + len(msg)
        self._wlen(rem)
        self._sendall(struct.pack("!H", len(topic))); self._sendall(topic)
        self._sendall(msg)
        self.last_io = time.ticks_ms()

    def ping(self):
        if not self.s: raise OSError("no socket")
        self._sendall(b"\xC0\x00")
        self.last_io = time.ticks_ms()

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
        except: pass

    def read(self):
        try:
            self._w(self.CMD_MEAS_HIGH_NC); time.sleep_ms(15)
            d = self.i2c.readfrom(self.addr, 6)
            if len(d) != 6: raise OSError(f"Expected 6 bytes, got {len(d)}")
            if _crc8(d[0:2]) != d[2] or _crc8(d[3:5]) != d[5]: raise OSError("CRC check failed")
            tr = (d[0] << 8) | d[1]; hr = (d[3] << 8) | d[4]
            tc = -45.0 + (175.0 * tr / 65535.0)
            rh = min(100.0, max(0.0, 100.0 * hr / 65535.0))
            self.last_t, self.last_h = tc, rh
            return tc, rh, True
        except OSError:
            return self.last_t, self.last_h, False

# ====== NET HELPERS (Pico 2 W safe) ==========================================
WLAN = network.WLAN(network.STA_IF)

def _wifi_off():
    try:
        if WLAN.active():
            WLAN.active(False)
            time.sleep_ms(150)
    except: pass

def wifi_connect(ssid=WIFI_SSID, pwd=WIFI_PASS, wdt=None, total_deadline_ms=20000):
    # Full cycle: power-save off, connect with bounded wait, WDT-safe.
    start = time.ticks_ms()
    _wifi_off()
    WLAN.active(True)
    # Disable power-save to reduce stalls on Pico W/2 W radios
    try: WLAN.config(pm=0xa11140)
    except: pass
    try: WLAN.disconnect()
    except: pass
    WLAN.connect(ssid, pwd)

    while True:
        if wdt: wdt.feed()
        st = WLAN.status()
        if WLAN.isconnected() and st == network.STAT_GOT_IP:
            return True
        if time.ticks_diff(time.ticks_ms(), start) > total_deadline_ms:
            return False
        time.sleep_ms(100)

def full_net_recycle(wdt=None):
    # Hard reset of radio and DHCP
    _wifi_off()
    time.sleep_ms(200)
    return wifi_connect(wdt=wdt)

def bounded_backoff_wait(base_ms, increment_ms, tries, max_ms, wdt=None):
    # Bounded exponential backoff with WDT feeding
    wait_ms = min(base_ms + increment_ms * tries, max_ms)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < wait_ms:
        if wdt:
            wdt.feed()
        time.sleep_ms(100)

# ====== HOME ASSISTANT MQTT DISCOVERY ========================================
def publish_ha_discovery(client, wdt=None):
    """Publish Home Assistant MQTT Discovery messages for auto-configuration"""
    if wdt: wdt.feed()
    
    # Get MAC address for unique ID
    mac = WLAN.config('mac')
    mac_str = ''.join('{:02x}'.format(b) for b in mac)
    
    # Device information (shared across all sensors)
    device_info = {
        "identifiers": [DEVICE_ID],
        "name": DEV_NAME,
        "model": "Pico 2 W + SHT30",
        "manufacturer": "Raspberry Pi",
        "sw_version": "1.0"
    }
    
    # Temperature sensor discovery config
    # Note: Use simple names - HA combines device name + sensor name for display
    temp_config = {
        "name": "Temperature",  # HA displays as: "Network Rack Temperature"
        "unique_id": f"{DEVICE_ID}_temperature",
        "device_class": "temperature",
        "state_topic": TEMP_STATE_TOPIC,
        "unit_of_measurement": "°F",
        "value_template": "{{ value }}",
        "availability_topic": AVAIL_TOPIC,
        "device": device_info
    }
    
    # Humidity sensor discovery config
    hum_config = {
        "name": "Humidity",  # HA displays as: "Network Rack Humidity"
        "unique_id": f"{DEVICE_ID}_humidity",
        "device_class": "humidity",
        "state_topic": HUM_STATE_TOPIC,
        "unit_of_measurement": "%",
        "value_template": "{{ value }}",
        "availability_topic": AVAIL_TOPIC,
        "device": device_info
    }
    
    # Publish discovery messages (retained)
    print("Publishing HA Discovery configs...")
    client.publish(TEMP_CONFIG_TOPIC, ujson.dumps(temp_config), retain=True)
    if wdt: wdt.feed()
    time.sleep_ms(50)
    
    client.publish(HUM_CONFIG_TOPIC, ujson.dumps(hum_config), retain=True)
    if wdt: wdt.feed()
    time.sleep_ms(50)
    
    # Publish online status
    client.publish(AVAIL_TOPIC, "online", retain=True)
    print("HA Discovery complete")

# ====== RUNTIME ==============================================================
# Watchdog (keep within max limit; we feed inside all loops)
WDT_SEC = 8  # Max allowed is ~8.4 seconds
wdt = WDT(timeout=8300)  # Use safe value under the 8388ms limit

def _now_ms():
    try: return time.ticks_ms()
    except: return int(time.time() * 1000)

def safe_reset():
    # Close sockets, turn off Wi-Fi, small delay, GC, then hard reset
    try: _wifi_off()
    except: pass
    gc.collect()
    time.sleep_ms(100)
    machine.reset()

def run():
    # I2C0 on Pico 2 W: SDA=GP0, SCL=GP1
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
    
    # Scan for I2C devices
    devices = i2c.scan()
    
    if 0x44 not in devices:
        print("SHT30 sensor not found at address 0x44!")
        print("Check wiring: VCC->3.3V, GND->GND, SDA->GP0, SCL->GP1")
        safe_reset()
    
    print("SHT30 sensor detected")
    sht = SHT30(i2c)

    print(f"Connecting to WiFi: {WIFI_SSID}")
    if not wifi_connect(wdt=wdt):
        print("WiFi connection failed, resetting...")
        safe_reset()
    
    print(f"WiFi connected: {WLAN.ifconfig()[0]}")

    # Keepalive tuned to publish cadence (MQTT requires < keepalive)
    KEEP = max(30, min(240, PUB_SEC // 2))
    client = MiniMQTT(DEVICE_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=KEEP)

    # Initial MQTT connect (WDT-safe)
    print(f"Connecting to MQTT broker: {MQTT_HOST}:{MQTT_PORT}")
    tries = 0
    while True:
        wdt.feed()
        try:
            client.connect(wdt=wdt)
            print("MQTT connected")
            
            # Publish Home Assistant Discovery configs
            publish_ha_discovery(client, wdt=wdt)
            
            # Bootstrap one reading
            t, h, _ = sht.read()
            tf = t * 9/5 + 32
            client.publish(TEMP_STATE_TOPIC, str(round(tf, 2)), retain=True)
            client.publish(HUM_STATE_TOPIC, str(round(h, 1)), retain=True)
            print(f"Initial reading: {round(tf,2)}F, {round(h,1)}%")
            break
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            tries += 1
            client.close()
            if tries % 2 == 1:
                # every other failure, recycle Wi-Fi
                full_net_recycle(wdt=wdt)
            # bounded backoff (WDT-safe)
            bounded_backoff_wait(2000, 300, tries, 6000, wdt)
            if tries > 6:
                print("Too many MQTT connection failures, resetting...")
                safe_reset()

    period = PUB_SEC * 1000
    next_tick = time.ticks_add(time.ticks_ms(), period)
    fail_pub = 0
    bad_reads = 0
    loop_count = 0

    print(f"Monitoring started (publish every {PUB_SEC}s)")
    print("=" * 50)

    while True:
        wdt.feed()
        now = time.ticks_ms()
        loop_count += 1

        # keepalive ping if idle
        if time.ticks_diff(now, client.last_io) > (KEEP * 1000 // 2):
            try:
                client.ping()
            except Exception as e:
                print("Ping failed, reconnecting...")
                # reconnect path
                client.close()
                if not full_net_recycle(wdt=wdt):
                    print("WiFi reconnection failed, resetting...")
                    safe_reset()
                tries = 0
                while True:
                    wdt.feed()
                    try:
                        client.connect(wdt=wdt)
                        publish_ha_discovery(client, wdt=wdt)
                        break
                    except Exception as e2:
                        tries += 1
                        client.close()
                        bounded_backoff_wait(1500, 250, tries, 5000, wdt)
                        if tries > 6:
                            print("Too many reconnection failures, resetting...")
                            safe_reset()

        # scheduled publish
        if time.ticks_diff(now, next_tick) >= 0:
            next_tick = time.ticks_add(next_tick, period)

            t, h, ok = sht.read()
            if not ok:
                bad_reads += 1
                print("Sensor read failed (CRC error)")
                if bad_reads >= 3:
                    # reinit I2C & sensor after repeated CRC/bus errors
                    print("Reinitializing I2C and sensor...")
                    try:
                        i2c.deinit()
                    except: pass
                    time.sleep_ms(50)
                    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
                    sht = SHT30(i2c)
                    bad_reads = 0
            else:
                bad_reads = 0

            tf = t * 9/5 + 32
            try:
                # Publish simple numeric values to state topics
                client.publish(TEMP_STATE_TOPIC, str(round(tf, 2)))
                client.publish(HUM_STATE_TOPIC, str(round(h, 1)))
                client.publish(AVAIL_TOPIC, "online", retain=True)
                print(f"Published: {round(tf,2)}F, {round(h,1)}% RH")
                fail_pub = 0
            except Exception as e:
                print(f"Publish failed: {e}")
                fail_pub += 1
                client.close()
                # GC + radio recycle + bounded backoff (all WDT-safe)
                gc.collect()
                if not full_net_recycle(wdt=wdt) or fail_pub >= 3:
                    print("Too many publish failures or WiFi issues, resetting...")
                    safe_reset()
                tries = 0
                while True:
                    wdt.feed()
                    try:
                        client.connect(wdt=wdt)
                        publish_ha_discovery(client, wdt=wdt)
                        break
                    except Exception as e2:
                        tries += 1
                        client.close()
                        t0 = time.ticks_ms()
                        while time.ticks_diff(time.ticks_ms(), t0) < min(1200 + 250*tries, 5000):
                            wdt.feed(); time.sleep_ms(100)
                        if tries > 6:
                            print("Too many reconnection attempts, resetting...")
                            safe_reset()

            gc.collect()

        # tiny cooperative sleep
        time.sleep_ms(30)
        # defensive: if Wi-Fi fell off in background, try to get it back
        if not WLAN.isconnected():
            print("WiFi connection lost, recovering...")
            if not full_net_recycle(wdt=wdt):
                print("WiFi recovery failed, resetting...")
                safe_reset()

print("=== Pico 2 W Network Rack Monitor ===")
print(f"Device: {DEV_NAME}")
print(f"ID: {DEVICE_ID}")

try:
    run()
except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C)")
except Exception as e:
    print(f"\nUnexpected error in main: {e}")
    import sys
    sys.print_exception(e)
    print("Performing safety reset...")
    # last-chance safety net
    safe_reset()
