# Recorded motion format

Each JSONL row looks like:

`{"t":0.02,"line":"PA1,20,1,4.000,1.000,0,1"}`

`t` is elapsed seconds since start of capture; `line` is a frame of the same versioned telemetry protocol sent by firmware. Use `--record recordings/hardware_session.jsonl` while streaming, then `--mode replay --file recordings/hardware_session.jsonl` to reproduce the same command inputs. Recordings and logs are excluded from Git by default, except for the small demonstration fixture.
