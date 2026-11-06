"""Gradual press-and-hold finger control for the grasp demo.

Hold 6 to keep tightening; hold Enter to ease open; let go to retain the
position target. This only changes requested finger displacement; an object
is never attached to the gripper. MuJoCo contacts determine whether it lifts.
"""
from dataclasses import dataclass

@dataclass
class HeldGrip:
    target: float = 0.025
    close_speed: float = 0.014   # meters of slide-joint travel per second
    open_speed: float = 0.009
    minimum: float = 0.0
    maximum: float = 0.025

    def step(self, dt: float, closing: bool, opening: bool) -> float:
        if not (0.0 <= dt <= 1.0):
            raise ValueError('Invalid elapsed time')
        # Opposing keys together: hold current target, no surprise jumps.
        if closing and not opening:
            self.target = max(self.minimum, self.target - self.close_speed*dt)
        elif opening and not closing:
            self.target = min(self.maximum, self.target + self.open_speed*dt)
        return self.target
