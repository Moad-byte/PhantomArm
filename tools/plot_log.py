"""Plot a session using optional matplotlib (pip install matplotlib)."""
import argparse,csv
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('file',nargs='?',default='logs/last_session.csv')
a=p.parse_args()
with Path(a.file).open(encoding='utf8',newline='') as f:r=list(csv.DictReader(f))
if not r:raise SystemExit('No log data')
try:import matplotlib.pyplot as plt
except ImportError:raise SystemExit('Run: pip install matplotlib')
t=[float(x['time_s']) for x in r]
plt.figure()
for label,key in [('X command','target_x'),('X actual','tip_x'),('Y command','target_y'),('Y actual','tip_y')]:
    plt.plot(t,[float(x[key]) for x in r],label=label)
plt.xlabel('Elapsed time (s)');plt.ylabel('Position (m)');plt.title('PhantomArm commanded vs simulated position')
plt.legend();plt.tight_layout();plt.show()
