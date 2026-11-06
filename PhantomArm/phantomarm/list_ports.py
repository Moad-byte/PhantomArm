"""Show available Windows COM ports before opening the serial receiver."""
def main():
    try:from serial.tools import list_ports
    except ImportError:
        print('Install requirements first: pip install -r requirements.txt')
        raise SystemExit(1)
    ports=list(list_ports.comports())
    if not ports:
        print('No serial devices found. Connect the Pico by DATA-capable USB cable.')
    for p in ports:
        print(f'{p.device:8s} | {p.description} | {p.hwid}')
if __name__=='__main__':main()
