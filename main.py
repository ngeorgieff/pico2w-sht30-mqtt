# Pico 2 W + SHT30 → MQTT (per-metric time/value) — optimized & hardened

import gc
import struct
import time

import network
import ujson
from machine import I2C, WDT, Pin, reset

# ====== USER SETTINGS (from secrets.py if present) ============================
try:
    import secrets  # WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD, DEVICE_NAME, PUBLISH_INTERVAL_SEC
    WIFI_SSID  = secrets.WIFI_SSID
    WIFI_PASS  = secrets.WIFI_PASSWORD
    MQTT_HOST  = secrets.MQTT_BROKER
    MQTT_PORT  = int(getattr(secrets, "MQTT_PORT", 1883))
    MQTT_USER  = getattr(secrets, "MQTT_USER", None)
    MQTT_PASS  = getattr(secrets, "MQTT_PASSWORD", None)
    DEV_NAME   = getattr(secrets, "DEVICE_NAME", "Environmental Monitor")
    PUB_SEC    = int(getattr(secrets, "PUBLISH_INTERVAL_SEC", 300))
except:
    # Fallbacks (only for first-boot testing)
    WIFI_SSID = "ssid"; WIFI_PASS = "pass"
    MQTT_HOST="10.0.0.10"; MQTT_PORT=1883; MQTT_USER=None; MQTT_PASS=None
    DEV_NAME="environmental_monitor"; PUB_SEC=300

# ====== SLUG + TOPICS ========================================================
def _slug(s):
    out = []
    s = s.lower()
    for ch in s:
        out.append(ch if ('a' <= ch <= 'z' or '0' <= ch <= '9' or ch == '_') else '_')
    val = "".join(out).strip('_')
    return val or "pico_sensor"

DEV_ID  = _slug(DEV_NAME)
AVAIL_T = DEV_ID + "/status"
TEMP_T  = DEV_ID + "/temperature_f"
HUM_T   = DEV_ID + "/humidity"

# ====== MINIMAL MQTT (QoS0, keepalive, no prints) ============================
try:
    import usocket as socket
except ImportError:
    import socket
try:
    import ussl as sslmod
except ImportError:
    sslmod = None

class MiniMQTT:
    def __init__(self, cid, host, port=1883, user=None, pwd=None, keepalive=60, ssl=False, ssl_params=None):
        self.cid = cid if isinstance(cid,(bytes,bytearray)) else str(cid).encode()
        self.host, self.port = host, port
        self.user = None if user is None else (user if isinstance(user,(bytes,bytearray)) else str(user).encode())
        self.pwd  = None if pwd  is None else (pwd  if isinstance(pwd,(bytes,bytearray))  else str(pwd).encode())
        self.keep = keepalive
        self.ssl  = ssl
        self.ssl_params = ssl_params or {}
        self.s = None
        self.last_io = 0

    def _sendall(self, b):
        mv = memoryview(b); n = 0
        while n < len(mv):
            w = self.s.write(mv[n:])
            if w is None:  # some ports return None on success
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

    def connect(self):
        addr = socket.getaddrinfo(self.host, self.port)[0][-1]
        self.s = socket.socket(); self.s.settimeout(10); self.s.connect(addr)
        if self.ssl:
            if sslmod is None: raise OSError("ssl")
            self.s = sslmod.wrap_socket(self.s, **self.ssl_params)

        flags = 0x02  # clean session
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

        t = self.s.read(1)
        if not t or t != b"\x20": raise OSError("connack")
        _ = self._rlen()
        ack = self.s.read(2)
        if not ack or ack[1] != 0x00: raise OSError("rc")
        self.last_io = time.ticks_ms()

    def publish(self, topic, msg, retain=False):
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
        self._sendall(b"\xC0\x00")
        self.last_io = time.ticks_ms()

    def close(self):
        try: self._sendall(b"\xE0\x00")
        except: pass
        try: self.s.close()
        except: pass
        self.s = None

# ====== I2C + SHT30 (CRC, short timeouts) ====================================
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
    CMD_MEAS_HIGH_NC = b"\x24\x00"   # single-shot, high rep, no clock stretch

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
            rh = 100.0 * hr / 65535.0
            if rh < 0: rh = 0.0
            elif rh > 100: rh = 100.0
            self.last_t, self.last_h = tc, rh
            return tc, rh, True
        except:
            return self.last_t, self.last_h, False

# ====== NET HELPERS ==========================================================
def wifi_connect(timeout_ms=15000):
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active(): wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASS)
        t0 = time.ticks_ms()
        while not wlan.isconnected() and time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            time.sleep_ms(200)
    return wlan if wlan.isconnected() else None

# ====== RUNTIME ==============================================================
# pre-alloc payload dicts to limit garbage
_temp_pl = {"time": 0, "value": 0.0}
_hum_pl  = {"time": 0, "value": 0.0}

# watchdog ~8s (feed at least twice per cycle)
WDT_SEC = 8
wdt = WDT(timeout=WDT_SEC * 1000)

def _now_ms():
    try: return time.ticks_ms()
    except: return int(time.time() * 1000)

def _publish_reading(cli, topic, pl, val):
    pl["time"] = _now_ms()
    pl["value"] = val
    cli.publish(topic, ujson.dumps(pl), retain=False)

def run():
    # I2C0 on Pico 2 W: SDA=GP0, SCL=GP1
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
    sht = SHT30(i2c)

    wlan = wifi_connect()
    if not wlan:
        reset()

    client = MiniMQTT(DEV_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=60)
    tries = 0
    while True:
        try:
            client.connect()
            client.publish(AVAIL_T, "online", retain=True)   # retained availability
            # send one retained sample so HA has immediate value
            t, h, ok = sht.read()
            tf = t * 9/5 + 32
            client.publish(TEMP_T, ujson.dumps({"time": _now_ms(), "value": round(tf,2)}), retain=True)
            client.publish(HUM_T,  ujson.dumps({"time": _now_ms(), "value": round(h,1)}),  retain=True)
            break
        except:
            tries += 1
            time.sleep_ms(300 + 200*tries)
            if tries > 5: reset()

    period = PUB_SEC * 1000
    last = time.ticks_ms()
    fail_pub = 0

    while True:
        # feed watchdog regularly
        wdt.feed()

        now = time.ticks_ms()

        # keepalive ping every ~25s of inactivity
        if time.ticks_diff(now, client.last_io) > 25000:
            try:
                client.ping()
            except:
                try: client.close()
                except: pass
                # reconnect fast without reboot
                wlan = wifi_connect()
                if not wlan: reset()
                client = MiniMQTT(DEV_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=60)
                tries = 0
                while True:
                    try:
                        client.connect()
                        client.publish(AVAIL_T, "online", retain=True)
                        break
                    except:
                        tries += 1
                        time.sleep_ms(250 + 200*tries)
                        if tries > 5: reset()

        # publish on schedule
        if time.ticks_diff(now, last) >= period:
            last = now
            t, h, ok = sht.read()
            tf = t * 9/5 + 32
            try:
                _publish_reading(client, TEMP_T, _temp_pl, round(tf, 2))
                _publish_reading(client, HUM_T,  _hum_pl,  round(h, 1))
                client.publish(AVAIL_T, "online", retain=True)
                fail_pub = 0
            except:
                fail_pub += 1
                # small GC & reconnect on trouble to avoid long gaps
                try: client.close()
                except: pass
                gc.collect()
                wlan = wifi_connect()
                if not wlan or fail_pub >= 3:
                    reset()
                client = MiniMQTT(DEV_ID, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, keepalive=60)
                tries = 0
                while True:
                    try:
                        client.connect()
                        client.publish(AVAIL_T, "online", retain=True)
                        break
                    except:
                        tries += 1
                        time.sleep_ms(250 + 200*tries)
                        if tries > 5: reset()

            # light GC each cycle to keep heap tidy
            gc.collect()

        # tiny sleep to yield
        time.sleep_ms(40)

try:
    run()
except KeyboardInterrupt:
    pass
except:
    reset()
