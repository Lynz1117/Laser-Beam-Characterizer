import threading, queue, json
try:
    import serial, serial.tools.list_ports
except Exception:
    serial = None

def list_ports():
    if serial is None: return []
    return [p.device for p in serial.tools.list_ports.comports()]

class SerialLink:
    """Simple newline-delimited JSON serial link with background RX thread"""
    def __init__(self, port: str, baud: int = 115200, timeout=0.1):
        if serial is None:
            raise RuntimeError("pyserial not installed; enable simulator or install pyserial")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.q = queue.Queue(maxsize=2000)
        self._alive = True
        self._t = threading.Thread(target=self._rx_loop, daemon=True)
        self._t.start()

    def close(self):
        self._alive = False
        try: self.ser.close()
        except Exception: pass

    def send(self, obj: dict):
        import json
        self.ser.write((json.dumps(obj) + "\n").encode("utf-8"))

    def recv_nowait(self):
        try: return self.q.get_nowait()
        except queue.Empty: return None

    def _rx_loop(self):
        buf = b""
        while self._alive:
            try:
                chunk = self.ser.read(1024)
                if not chunk: continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.strip()
                    if not s: continue
                    try:
                        pkt = json.loads(s.decode("utf-8", "replace"))
                        self.q.put_nowait(pkt)
                    except Exception:
                        # drop malformed line
                        pass
            except Exception:
                # transient serial error; continue trying
                pass
