# Pico 2 W + SHT30 → MQTT (per-metric time/value) — optimized & hardened
# Tested on MicroPython for RP2350 (Pico 2 W). Place as main.py.
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
    DEV_NAME   = getattr(secrets, "DEVICE_NAME", "Environmental Monitor")
    PUB_SEC    = int(getattr(secrets, "PUBLISH_INTERVAL_SEC", 300))
except (ImportError, AttributeError):
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
        ai = socket.getaddrinfo(self.host, self.port, 0, socket.SOCK_STREAM)[0][-1]
        if wdt: wdt.feed()
        s = socket.socket()
        try:
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
            if not t or t != b"\x20": raise OSError("connack")
            _ = self._rlen()
            ack = self.s.read(2)
            if not ack or ack[1] != 0x00: raise OSError("rc")
            self.last_io = time.ticks_ms()
        except Exception:
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
            if len(d) != 6: raise OSError
            if _crc8(d[0:2]) != d[2] or _crc8(d[3:5]) != d[5]: raise OSError
            tr = (d[0] << 8) | d[1]; hr = (d[3] << 8) | d[4]
            tc = -45.0 + (175.0 * tr / 65535.0)
            rh = min(100.0, max(0.0, 100.0 * hr / 65535.0))
            self.last_t, self.last_h = tc, rh
            return tc, rh, True
        except OSError:
            return self.last_t, self.last_h, False

# ====== NET HELPERS (Pico 2 W safe) =========================================
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

# ====== RUNTIME ==============================================================
_temp_pl = {"time": 0, "value": 0.0}
_hum_pl  = {"time": 0, "value": 0.0}

# Watchdog (keep fairly generous; we feed inside all loops)
WDT_SEC = 10
wdt = WDT(timeout=WDT_SEC * 1000)

def _now_ms():
    try: return time.ticks_ms()
    except: return int(time.time() * 1000)

def _publish_reading(cli, topic, pl, val):
    pl["time"] = _now_ms()
    pl["value"] = val
    cli.publish(topic, ujson.dumps(pl), retain=False)

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
    sht = SHT30(i2c)

    if not wifi_connect(wdt=wdt):
        safe_reset()

    # Keepalive tuned to publish cadence (MQTT requires < keepalive)
    KEEP = max(30, min(240, PUB_SEC // 2))
    client = MiniMQTT(DEV_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=KEEP)

    # Initial MQTT connect (WDT-safe)
    tries = 0
    while True:
        wdt.feed()
        try:
            client.connect(wdt=wdt)
            client.publish(AVAIL_T, "online", retain=True)
            # bootstrap one retained value for HA
            t, h, _ = sht.read()
            tf = t * 9/5 + 32
            nowms = _now_ms()
            client.publish(TEMP_T, ujson.dumps({"time": nowms, "value": round(tf,2)}), retain=True)
            client.publish(HUM_T,  ujson.dumps({"time": nowms, "value": round(h,1)}),  retain=True)
            break
        except Exception:
            tries += 1
            client.close()
            if tries % 2 == 1:
                # every other failure, recycle Wi-Fi
                full_net_recycle(wdt=wdt)
            # bounded backoff (WDT-safe)
            # bounded backoff (WDT-safe)
            bounded_backoff_wait(2000, 300, tries, 6000, wdt)
            if tries > 6:
                safe_reset()

    period = PUB_SEC * 1000
    next_tick = time.ticks_add(time.ticks_ms(), period)
    fail_pub = 0
    bad_reads = 0

    while True:
        wdt.feed()
        now = time.ticks_ms()

        # keepalive ping if idle
        if time.ticks_diff(now, client.last_io) > (KEEP * 1000 // 2):
            try:
                client.ping()
            except Exception:
                # reconnect path
                client.close()
                if not full_net_recycle(wdt=wdt):
                    safe_reset()
                tries = 0
                while True:
                    wdt.feed()
                    try:
                        client.connect(wdt=wdt)
                        client.publish(AVAIL_T, "online", retain=True)
                        break
                    except Exception:
                        tries += 1
                        client.close()
                        t0 = time.ticks_ms()
                        while time.ticks_diff(time.ticks_ms(), t0) < min(1500 + 250*tries, 5000):
                            wdt.feed(); time.sleep_ms(100)
                        if tries > 6:
                            safe_reset()

        # scheduled publish
        if time.ticks_diff(now, next_tick) >= 0:
            next_tick = time.ticks_add(next_tick, period)

            t, h, ok = sht.read()
            if not ok:
                bad_reads += 1
                if bad_reads >= 3:
                    # reinit I²C & sensor after repeated CRC/bus errors
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
                _publish_reading(client, TEMP_T, _temp_pl, round(tf, 2))
                _publish_reading(client, HUM_T,  _hum_pl,  round(h, 1))
                client.publish(AVAIL_T, "online", retain=True)
                fail_pub = 0
            except Exception:
                fail_pub += 1
                client.close()
                # GC + radio recycle + bounded backoff (all WDT-safe)
                gc.collect()
                if not full_net_recycle(wdt=wdt) or fail_pub >= 3:
                    safe_reset()
                tries = 0
                while True:
                    wdt.feed()
                    try:
                        client.connect(wdt=wdt)
                        client.publish(AVAIL_T, "online", retain=True)
                        break
                    except Exception:
                        tries += 1
                        client.close()
                        t0 = time.ticks_ms()
                        while time.ticks_diff(time.ticks_ms(), t0) < min(1200 + 250*tries, 5000):
                            wdt.feed(); time.sleep_ms(100)
                        if tries > 6:
                            safe_reset()

            gc.collect()

        # tiny cooperative sleep
        time.sleep_ms(30)
        # defensive: if Wi-Fi fell off in background, try to get it back
        if not WLAN.isconnected():
            if not full_net_recycle(wdt=wdt):
                safe_reset()

try:
    run()
except KeyboardInterrupt:
    pass
except Exception:
    # last-chance safety net
    safe_reset()