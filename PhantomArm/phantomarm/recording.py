"""Line-oriented motion capture and replay format."""
import json
import time
from pathlib import Path
from .telemetry import Sample,format_line

class Recorder:
    def __init__(self,path):
        p=Path(path)
        p.parent.mkdir(parents=True,exist_ok=True)
        self.out=p.open('w',encoding='utf-8')
        self.t0=time.monotonic()
    def write(self,sample:Sample,at:float):
        self.out.write(json.dumps({'t':round(max(0,at-self.t0),4), 'line':format_line(sample)})+'\n')
    def close(self):
        self.out.close()
