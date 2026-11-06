import pytest
from phantomarm.telemetry import Sample,parse_line,format_line,MalformedTelemetry

def test_roundtrip():
    s=Sample(100,4,-12.5,21.25,True,True)
    assert parse_line(format_line(s))==s

@pytest.mark.parametrize('line',[
    '', 'PA1,1,2,3', 'PA2,1,2,3,4,0,1','PA1,-1,2,3,4,0,1',
    'PA1,1,2,nan,4,0,1','PA1,1,2,181,4,0,1','PA1,1,2,3,4,2,1',
    'PA1,1,2,3,4,0,yes','PA1,1,-2,3,4,0,1',
])
def test_reject_bad(line):
    with pytest.raises(MalformedTelemetry):parse_line(line)

def test_invalid_sensor_flag_is_parseable():
    assert not parse_line('PA1,25,4,0,0,0,0').valid
