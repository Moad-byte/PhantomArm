"""Optional CSV run logs for plots and hardware evidence."""
import csv
from pathlib import Path

class SessionLog:
    def __init__(self,path):
        dest=Path(path)
        dest.parent.mkdir(parents=True,exist_ok=True)
        self.file=dest.open('w',newline='',encoding='utf-8')
        self.writer=csv.writer(self.file)
        self.writer.writerow(['time_s','mode','state','roll_deg','pitch_deg','target_x','target_y','target_z','tip_x','tip_y','tip_z','error_m'])
    def row(self,t,mode,state,s,goal,tip,error):
        self.writer.writerow([round(t,3),mode,state,
                              round(s.roll_deg,3) if s else '',round(s.pitch_deg,3) if s else '',
                              round(goal.x,4),round(goal.y,4),round(goal.z,4),
                              *[round(float(v),4) for v in tip],round(error,4)])
        self.file.flush()
    def close(self):self.file.close()
