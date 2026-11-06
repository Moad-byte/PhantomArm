# Optional Ubuntu + VMware fallback (not the recommended path)

**Use native Windows first.** MuJoCo supports a Windows Python wheel and an OpenGL viewer; the firmware uses a native Windows COM port and Arduino IDE. A VMware Ubuntu guest adds virtual GPU and USB passthrough complexity without adding robotics features.

If Windows rendering is genuinely blocked:

1. Install VMware and Ubuntu Desktop guest as supported by your hardware and license.
2. Enable **3D accelerated graphics** for the guest and install VMware Tools / open-vm-tools.
3. Assign at least 4 virtual CPU cores, 6 GB RAM, 25 GB disk if the host can spare them.
4. From Ubuntu guest terminal run `bash scripts/setup_ubuntu.sh`.
5. Run `./.venv/bin/python -m phantomarm.verify_model` and then `./.venv/bin/python -m phantomarm.app --mode demo`.
6. If forwarding the Pico, select USB > attach **Pico 2 W to the guest**. Linux port may appear `/dev/ttyACM0`. Allow serial permissions using `sudo usermod -a -G dialout "$USER"` then log out and log in.
7. Run `./.venv/bin/python -m phantomarm.app --mode serial --port /dev/ttyACM0`.

If 3D graphics under the VM are unstable, do **not** spend the entire project timeline fixing the guest. Windows-native remains the fast path. Real hardware can alternatively stream to host Windows while the Linux guest only runs demo mode.
