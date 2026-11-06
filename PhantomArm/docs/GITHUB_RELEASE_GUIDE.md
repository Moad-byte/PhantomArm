# GitHub publishing and release procedure

## Start version control after local tests

Open PowerShell in the project root:

```powershell
git init
git add .
git commit -m "feat: original PhantomArm teleoperation model and controls"
```

Create a **blank** repository called `PhantomArm` on GitHub (no auto-generated README, since ours already exists). Add your own GitHub URL:

```powershell
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/PhantomArm.git
git push -u origin main
```

## Suggested commit milestones

1. `feat: original 4-axis MJCF arm and analytic IK`
2. `feat: Pico 2 W MPU6050 orientation transmitter`
3. `feat: live USB telemetry and motion mapping`
4. `test: watchdog, ESTOP, replay and validation evidence`
5. `docs: wiring, demo media, verified test results`

## What to publish

- `assets/phantom_arm.xml` — original editable robot geometry
- `phantomarm/` — Python simulation and control pipeline
- `firmware/` — embedded C++
- `tests/` — regression suite
- `docs/` — full setup, architecture and wiring
- `media/` — **your** actual hardware photo / screen-recorded GIF and videos

Do not commit `.venv`, binary SDK packages, personal Windows COM settings, huge videos or secrets.

## First release

When all acceptance checks pass, create a GitHub Release (for example `v1.0.0`) with real verification outcomes, a short demo video link, and a note explaining physical sensor controller + virtual robotic arm. Don't invent benchmark totals, real robot deployment or safety certification.

You can include three media pieces for LinkedIn: real controller + real simulator screen, short GIF of arm movement, and one architecture diagram.
