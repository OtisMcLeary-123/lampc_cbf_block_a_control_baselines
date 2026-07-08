from __future__ import annotations

import numpy as np

from .scenario import Scenario


def clip_norm(values: np.ndarray, max_norm: float) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    norm = np.linalg.norm(arr, axis=-1, keepdims=True)
    scale = np.minimum(1.0, max_norm / np.maximum(norm, 1e-12))
    return arr * scale


def step_state(states: np.ndarray, controls: np.ndarray, scenario: Scenario) -> np.ndarray:
    dt = scenario.simulation.dt
    controls = clip_norm(controls, scenario.robot.max_accel)
    pos = states[..., :2]
    vel = states[..., 2:]
    next_pos = pos + vel * dt + 0.5 * controls * dt * dt
    next_vel = clip_norm(vel + controls * dt, scenario.robot.max_speed)
    return np.concatenate([next_pos, next_vel], axis=-1)


def nominal_pd_control(state: np.ndarray, scenario: Scenario) -> np.ndarray:
    pos = state[:2]
    vel = state[2:]
    to_target = scenario.robot.target - pos
    desired_vel = clip_norm(to_target[None, :] * 0.9, scenario.robot.max_speed)[0]
    control = 2.2 * (desired_vel - vel)
    return clip_norm(control[None, :], scenario.robot.max_accel)[0]
