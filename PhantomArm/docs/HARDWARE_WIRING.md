# Hardware wiring and Arduino firmware

## Minimal hardware

- Raspberry Pi Pico **2 W** (the same board used in StabilityGuard).
- MPU6050 GY-521 breakout board.
- Breadboard, data-capable USB cable, 4 jumper wires.
- Optional momentary push button and 2 LEDs with individual 220–330 Ω resistors.

## Wiring table

| Component signal | Pico 2 W physical pin | Pico GPIO | Notes |
|---|---:|---|---|
| MPU6050 VCC | 36 | 3V3(OUT) | **3.3 V**, not VBUS/5 V |
| MPU6050 GND | 38 (or 8) | GND | Common ground |
| MPU6050 SDA | 6 | GP4 | I²C0 SDA |
| MPU6050 SCL | 7 | GP5 | I²C0 SCL |
| Button leg 1 | 20 | GP15 | Internal pull-up enabled |
| Button leg 2 | 18 (or any GND) | GND | Press connects GP15 to GND |
| Green LED anode | 21 | GP16 | Through 220–330 Ω resistor |
| Green LED cathode | 23 | GND | Optional |
| Red LED anode | 22 | GP17 | Through 220–330 Ω resistor |
| Red LED cathode | 23 | GND | Optional |

**Power off and disconnect USB while rewiring.** Verify your MPU6050 module labels. Some breakouts support other voltage inputs, but always use 3V3 for this build. Do not connect two GPIO pins together. Never connect an LED directly without a resistor.

You may reuse the MPU6050 and components from StabilityGuard after making a copy of the hardware demonstration video.

## Firmware upload

1. Install Arduino IDE 2: https://www.arduino.cc/en/software
2. Board Manager: install the **Raspberry Pi Pico/RP2040/RP2350** core from Earle Philhower as documented at https://arduino-pico.readthedocs.io/en/latest/install.html
3. Library Manager: install `Adafruit MPU6050`, plus its dependency `Adafruit Unified Sensor` and `Adafruit BusIO` if prompted. https://github.com/adafruit/Adafruit_MPU6050
4. Open `firmware/PhantomArm_Pico2W/PhantomArm_Pico2W.ino`.
5. Tools > Board > Raspberry Pi Pico 2 W, then select the device's COM port.
6. Upload. After the sketch starts, use Serial Monitor at **115200 baud** to check 50 Hz `PA1` output.
7. If Arduino can't find the board: reconnect with BOOTSEL pressed to enter bootloader mode and retry using Arduino-Pico upload instructions.

## What firmware does

- Reads accelerometer + gyroscope over GP4/GP5 I²C at 50 Hz.
- Uses a complementary orientation filter with accelerometer correction.
- Debounces physical button GP15.
- Sends explicit sensor-valid flag and monotonic sequence counter.
- If MPU6050 is missing, sends `valid=0`; host should HOLD; retries sensor init.
- LEDs: green = sensor readings valid; red = invalid sensor (optional wires).

It does **not** infer 3D translation or precise hand position: only two tilt angles. Sudden linear acceleration can affect estimated tilt. Press `C` to calibrate a stable neutral orientation on the laptop. No wireless link is required, so it can be tested rapidly via USB serial.

## Hardware safety

This is a simulation controller. `E` is only a **software** emergency stop for the virtual robot. It is not suitable for controlling powerful physical actuators, hazardous equipment, or people-facing robots. Do not represent it as a certified safety system.
