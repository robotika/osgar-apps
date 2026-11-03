"""Wire-distance validity checks shared by offline audits and pose estimation."""

from osgar.platforms.matty import RobotStatus


def signed_delta(current, previous):
    return [(int(c) - int(p) + 32768) % 65536 - 32768 for c, p in zip(current, previous)]


def validate_bounds(max_gap, max_wheel_speed):
    if not (0 < max_gap < float('inf') and 0 < max_wheel_speed < float('inf')
            and max_gap * max_wheel_speed * 1000 + 2 < 32768):
        raise ValueError('Positive finite bounds must limit wheel travel below half the 16-bit modulus')


def wheel_interval(previous, current, dt, previous_flags, flags, *, max_gap=.5, max_wheel_speed=1.5):
    validate_bounds(max_gap, max_wheel_speed)
    delta = signed_delta(current, previous)
    reasons = []
    if dt <= 0:
        reasons.append('nonpositive_interval')
    elif dt > max_gap:
        reasons.append('time_gap')
    if flags & RobotStatus.ERROR_ENCODER.value:
        reasons.append('encoder_error')
    if previous_flags & RobotStatus.ERROR_ENCODER.value:
        reasons.append('after_encoder_error')
    if (flags | previous_flags) & RobotStatus.ERROR_POWER.value:
        reasons.append('power_error')
    if dt > 0 and max(abs(x) for x in delta) > max_wheel_speed * 1000 * dt + 2:
        reasons.append('encoder_jump_or_reset')
    return delta, reasons
