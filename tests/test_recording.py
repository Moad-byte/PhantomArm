from phantomarm.recording import Recorder
from phantomarm.sources import ReplaySource
from phantomarm.telemetry import Sample

def test_recording_roundtrip(tmp_path,monkeypatch):
    # Real wallclock timestamps are avoided in this test by explicitly passing
    # recorder-relative time from the freshly initialized Recorder object.
    p=tmp_path/'session.jsonl';rec=Recorder(p)
    s=Sample(20,1,2,3,False,True)
    rec.write(s,rec.t0+.02)
    rec.close()
    replay=ReplaySource(p)
    got=replay.poll(replay.t0+.05)
    assert got==s
    assert replay.poll(replay.t0+.1) is None
