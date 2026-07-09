#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# ///
# ─── How to run ───
# uv run qa/test_motion_parity.py
"""Static parity guard for GMod motion-controller force semantics."""

from pathlib import Path
from typing import Final


ROOT: Final[Path] = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


motion: Final[str] = read("vphysics_box3d/vbox_controller_motion.cpp")
obj: Final[str] = read("vphysics_box3d/vbox_object.cpp")

require(
    "SIM_GLOBAL_FORCE" in motion and "SIM_LOCAL_FORCE" in motion,
    "motion controller must handle force result modes",
)
require(
    "bLocalResult" in motion,
    "motion controller must branch on local vs global result coordinate space",
)
require(
    "WorldToLocalVector(&angLocalAngular" in motion,
    "SIM_GLOBAL_ACCELERATION angular velocity must convert world input to local AddVelocity input",
)
require(
    "ApplyTorqueCenter(angWorldAngular" in motion,
    "SIM_GLOBAL_FORCE angular impulse must be applied in world space without local rotation",
)
require(
    "always in the object's local space" not in motion,
    "stale local-only angular comment indicates GMod SIM_GLOBAL_FORCE parity bug remains",
)

velocity_fn: Final[str] = obj.split("void Box3DPhysicsObject::CalculateVelocityOffset", 1)[1].split(
    "float Box3DPhysicsObject::CalculateLinearDrag", 1
)[0]
require(
    "!IsMoveable()" in velocity_fn,
    "CalculateVelocityOffset must return zero for motion-disabled objects",
)
require(
    "vec3_origin" in velocity_fn,
    "motion-disabled CalculateVelocityOffset must explicitly zero both output vectors",
)

print("motion parity guard passed")
