"""Robust host-side telemetry protocol used by firmware and mock sources."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Sample:
    device_ms: int
    seq: int
    roll_deg: float
    pitch_deg: float
    button: bool
    valid: bool
    # PA2 held-button states; defaults keep PA1, replay, and demo compatible.
    z_up: bool = False
    z_down: bool = False
    grip_close: bool = False
    grip_open: bool = False
    protocol: str = 'PA1'

class MalformedTelemetry(ValueError):
    pass


def parse_line(line: str) -> Sample:
    chunks = line.strip().split(',')
    if len(chunks) == 10 and chunks[0] == 'PA2':
        # PA2,ms,seq,roll,pitch,z_up,z_down,grip_close,grip_open,valid
        try:
            ms, seq = int(chunks[1]), int(chunks[2])
            roll, pitch = float(chunks[3]), float(chunks[4])
            if any(flag not in ('0', '1') for flag in chunks[5:10]):
                raise ValueError('PA2 flags must be 0 or 1')
        except (ValueError, OverflowError) as exc:
            raise MalformedTelemetry('Invalid PA2 telemetry') from exc
        if ms < 0 or seq < 0 or not all(map(math.isfinite, (roll, pitch))):
            raise MalformedTelemetry('Non-finite or negative telemetry')
        if abs(roll) > 180 or abs(pitch) > 180:
            raise MalformedTelemetry('Angles outside protocol limits')
        return Sample(ms, seq, roll, pitch, False, chunks[9] == '1',
                      z_up=chunks[5] == '1', z_down=chunks[6] == '1',
                      grip_close=chunks[7] == '1', grip_open=chunks[8] == '1',
                      protocol='PA2')
    if len(chunks) != 7 or chunks[0] != 'PA1':
        raise MalformedTelemetry('Expected PA1,ms,seq,roll,pitch,button,valid')
    try:
        ms, seq = int(chunks[1]), int(chunks[2])
        roll, pitch = float(chunks[3]), float(chunks[4])
        if chunks[5] not in ('0','1') or chunks[6] not in ('0','1'):
            raise ValueError('button/valid must be 0 or 1')
        button, valid = chunks[5]=='1', chunks[6]=='1'
    except (ValueError, OverflowError) as exc:
        raise MalformedTelemetry('Invalid numeric telemetry') from exc
    if ms < 0 or seq < 0 or not all(map(math.isfinite,(roll,pitch))):
        raise MalformedTelemetry('Non-finite or negative telemetry')
    if abs(roll)>180 or abs(pitch)>180:
        raise MalformedTelemetry('Angles outside protocol limits')
    return Sample(ms,seq,roll,pitch,button,valid)


def format_line(s: Sample) -> str:
    if s.protocol == 'PA2':
        return (f'PA2,{s.device_ms},{s.seq},{s.roll_deg:.3f},{s.pitch_deg:.3f},'
                f'{int(s.z_up)},{int(s.z_down)},{int(s.grip_close)},'
                f'{int(s.grip_open)},{int(s.valid)}')
    return f'PA1,{s.device_ms},{s.seq},{s.roll_deg:.3f},{s.pitch_deg:.3f},{int(s.button)},{int(s.valid)}'
