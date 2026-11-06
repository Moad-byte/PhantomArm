# Third-party tools and references

The original PhantomArm source code, analytic IK implementation, C++ transmitter, diagrams and MJCF robot scene are authored for this project. The following are third-party tools **not copied into the project ZIP**:

1. **Google DeepMind MuJoCo** (Apache-2.0): https://github.com/google-deepmind/mujoco and official viewer API https://mujoco.readthedocs.io/en/stable/python.html . The installed package provides the physics engine and visualization.
2. **Adafruit MPU6050 Arduino library** (BSD license; check upstream distribution): https://github.com/adafruit/Adafruit_MPU6050 . Reads I²C sensor data.
3. **Raspberry Pi official Pico 2 pinout**: https://datasheets.raspberrypi.com/pico/Pico-2-Pinout.pdf . Verify GP4/5, GP15, GP16/17 and 3V3 wiring.
4. **Arduino-Pico Wire documentation**: https://arduino-pico.readthedocs.io/en/latest/wire.html . Explains setSDA/setSCL on Pico.
5. **PySerial**: https://pyserial.readthedocs.io/en/latest/shortintro.html . Windows COM-port support.
6. **pytest**: https://docs.pytest.org/ . Software testing.

No copied CAD, third-party robot artwork, licensed mesh or unverified marketing photos are included. Screenshots and media placeholders must be replaced with your own actual demonstrations.
