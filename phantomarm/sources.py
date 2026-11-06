"""Sensor input implementations: demo, physical COM port, JSONL replay."""
import json
import math
from pathlib import Path
import time
from .telemetry import Sample, parse_line, MalformedTelemetry

class DemoSource:
    """Synthetic MPU6050 data, stationary for isolated controls or swept for showcase."""
    def __init__(self, pattern="still"):
        if pattern not in ("still", "sweep"):
            raise ValueError("pattern must be still or sweep")
        self.pattern=pattern
        self.t0=time.monotonic()
        self.seq=0
        self.last_send=0.0

    def poll(self, now:float):
        if now-self.last_send<0.02:
            return None
        self.last_send=now
        t=now-self.t0
        self.seq+=1
        if self.pattern == "sweep":
            roll=25*math.sin(t*0.85)
            pitch=20*math.sin(t*0.54)
        else:
            roll=pitch=0.0
        return Sample(int(t*1000),self.seq,roll,pitch,False,True)
    def close(self): pass

class SerialSource:
    def __init__(self,port,baud=115200):
        try: import serial
        except ImportError as exc:
            raise RuntimeError('Install pyserial: pip install pyserial') from exc
        self.connection=serial.Serial(port,baudrate=baud,timeout=0)
        self._bytes=b''
        self.bad_lines=0
        self.last_error=''

    def poll(self,now):
        pending=self.connection.read(min(self.connection.in_waiting or 0,2048))
        if pending: self._bytes+=pending
        if len(self._bytes)>8192:
            self._bytes=b''
            self.bad_lines+=1
            self.last_error='Overlong serial buffer'
            return None
        latest=None
        while b'\n' in self._bytes:
            line,self._bytes=self._bytes.split(b'\n',1)
            try: latest=parse_line(line.decode('ascii').strip())
            except (MalformedTelemetry,UnicodeDecodeError) as exc:
                self.bad_lines+=1
                self.last_error=str(exc)
        return latest

    def close(self):self.connection.close()

class ReplaySource:
    def __init__(self,path):
        self.rows=[]
        for line in Path(path).read_text(encoding='utf-8').splitlines():
            if line.strip():
                d=json.loads(line)
                self.rows.append((float(d['t']),parse_line(d['line'])))
        if not self.rows:raise ValueError('Replay is empty')
        self.index=0
        self.t0=time.monotonic()
        self.last_time=-1
        for t,_ in self.rows:
            if t<self.last_time:raise ValueError('Replay timestamps not sorted')
            self.last_time=t

    def poll(self,now):
        elapsed=now-self.t0
        latest=None
        while self.index<len(self.rows) and self.rows[self.index][0]<=elapsed:
            latest=self.rows[self.index][1]
            self.index+=1
        return latest
    def close(self):pass
