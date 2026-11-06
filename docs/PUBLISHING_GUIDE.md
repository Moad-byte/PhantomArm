# PhantomArm — first public GitHub release guide

## Files to use

1. Extract `PhantomArm_GitHub_Release_v1.0.0.zip` locally.
2. Open the contained `PhantomArm` directory (the folder with `README.md`, `.gitignore`, and `LICENSE`).
3. Do **not** upload the ZIP itself as your GitHub repository contents. Upload its files preserving directories.
4. The project includes the actual hardware-demo MP4, a four-frame GIF README banner, a social-preview JPG, and a separately labeled industrial concept illustration.

## New GitHub repository settings

- Repository name: `PhantomArm`
- Description: `IMU-controlled robotic arm teleoperation with Raspberry Pi Pico 2 W, MPU6050, Python and MuJoCo`
- Visibility: `Public` (choose Private if you do not want public source access).
- Add README: **No** (already exists).
- Add .gitignore: **No** (already exists).
- Choose license: **No** (MIT file already exists).
- Default branch: `main`.
- About/topics: `robotics`, `embedded-systems`, `teleoperation`, `raspberry-pi-pico`, `mpu6050`, `mujoco`, `python`, `arduino`, `inverse-kinematics`, `robotic-arm`, `human-machine-interface`.
- Social preview image: `media/github_social_preview.jpg` (GitHub repository **Settings → General → Social preview**).

## Upload via Git (recommended for retaining directory structure)

From a PowerShell terminal opened in the extracted `PhantomArm` folder:

```powershell
git init
git branch -M main
git add .
git status
git commit -m "release: PhantomArm physical teleoperation v1.0.0"
git remote add origin https://github.com/YOUR_USERNAME/PhantomArm.git
git push -u origin main
```

Replace `YOUR_USERNAME` with your own GitHub username. If a remote named `origin` already exists, inspect `git remote -v` before adding another. Verify `git status` before committing to prevent unintended data upload.

Alternatively, create the blank repository at https://github.com/new and upload the **contents** of the extracted `PhantomArm` directory via **Add file → Upload files**. Browser uploads accept up to 100 files per batch, with a 25 MiB per-file limit; the demo MP4 currently fits under that limit. Ensure the `.gitignore` is included.

## About area

- Add the short description shown above.
- Topics as listed above (you can use 8–12 topics).
- Website: leave blank unless you have a real demo or personal portfolio URL.
- Consider enabling Issues; Discussions/Wiki/Projects are optional and can remain disabled.

## First release — v1.0.0

Go to the repository → **Releases** → **Draft a new release**.

- New tag: `v1.0.0` (from `main`).
- Release title: `PhantomArm v1.0.0 — Hardware-Controlled Pick-and-Place`.
- Type: stable release, **not** pre-release (this is a functioning prototype, not a certified industrial system).
- Set as latest release: **Yes**.
- Add the release description below, replacing `YOUR_USERNAME` in the two media links. GitHub automatically generates Source code ZIP/TAR archives for the release.
- Optionally upload `PhantomArm_GitHub_Release_v1.0.0.zip` as a convenience asset (optional; repository source is already downloadable).

### Pasteable release description

```markdown
## First public release

PhantomArm is an IMU-controlled robotic-arm teleoperation prototype connecting a **Raspberry Pi Pico 2 W + MPU6050 + four physical buttons** to a **Python-controlled MuJoCo simulation** over USB serial.

### Features
- Real-time pitch/roll-based X/Y arm motion with initial orientation calibration and smoothing.
- Hold-to-move Z-up/Z-down buttons and hold-to-close/open gripper buttons.
- Analytic inverse kinematics, workspace constraints, and gradual motor targets.
- Physical contact-based cube grasp, lift, transport and placement between platforms.
- Insert-key cube-only respawn without resetting the controlled arm.
- Keyboard fallback, serial telemetry validation, and test/diagnostic tooling.

### Media
- [Full hardware-operated demonstration](https://github.com/YOUR_USERNAME/PhantomArm/blob/main/media/phantomarm_hardware_demo.mp4)
- [Physical controller photograph](https://github.com/YOUR_USERNAME/PhantomArm/blob/main/media/phantomarm_hardware_setup.jpg)

### Notes
The physical hardware is the **input controller**; the robot arm is **simulated** in MuJoCo. This is an educational/research prototype, not a certified industrial safety system. Off-angle cube grasps can occasionally cause a mechanical contact jam.
```

## LinkedIn post packaging

Suggested media: (1) `media/phantomarm_industrial_concept.png` as an explicitly *conceptual application illustration*, (2) `media/phantomarm_hardware_demo.mp4` as authentic evidence, and (3) `media/phantomarm_hardware_setup.jpg` for a photo post or carousel. Lead with the real demo if you want to emphasize engineering credibility, then mention the industrial concept as an inspiration/use case rather than a completed factory system.

Do not publish manufactured performance statistics or claim that the simulated robot arm is physical.

## Final verification

- On GitHub's repository front page, confirm the README GIF animates at the top.
- Confirm the JPG and MP4 links open.
- Verify `sketch/sketch.ino` is the actual four-button PA2 firmware.
- Confirm `.venv`, caches, personal paths, tokens, and serial logs are absent from the committed files.
- Run tests on the checked-out project before publishing fresh pass counts.
