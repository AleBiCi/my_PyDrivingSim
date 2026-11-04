import math
from vehicle_model.motor_model import motor_model

# ----------------------------------------------------------------
## Function purpose: generic 4WD driveline (one engine -> centre differential -> front and rear axle differentials),
#                    used when vehicle_data.transmission.drive_layout == "driveline_4wd".
#                    The Chimera Evo ("per_wheel_motors", one motor per wheel) does not use this file.
#
#  References (Dynamics of Vehicles, VD-Lecture-05b, Biral, PDF page numbers):
#    p.37-38  Willis formula, open differential: omega_H = (omega_a + omega_b)/2
#    p.48/51  torque bias of a differential with internal friction: T_slow - T_fast = eps*|T_H|,
#             eps = (1 - eta)/(1 + eta) = (TBR - 1)/(TBR + 1), TBR = torque bias ratio = 1/eta
#    p.54     any differential: the internal torque acts against the speed difference, sign(delta_omega)
#    p.56-58  open / locked / limited-slip characteristic
#    p.59     geared (Torsen) LSD: bias proportional to |T_H|, different on power-on and power-off
#    p.60     clutch-pack LSD: preload Tp + ramp, bias = Tp + eps*|T_H|
#  and Ale's notes "Differential implementation" (p.2): delta_T = min(|delta_T_lock|, |delta_T_LSD(T_H)|),
#  delta_T_lock = K_dif*delta_omega (no reduction of the number of states). NOTE: the notes give the larger
#  torque to the faster wheel; here the slower wheel gets the larger torque, as in Biral p.48/54.
#
#  Modelling choices (NOT IN SLIDES):
#    - "locked" is approximated by a stiff internal damper K_dif*|delta_omega| with no torque limit, so a locked
#      differential still allows a small speed difference (creep). Explicit Euler stability needs
#      K_dif*dt/iwa < 2 (axle) -> TransmissionData.k_dif is chosen per parameter set (TODO(dev) values).
#    - The engine is the sim motor map (motor_model, two "motors" of maxTorque and maxPower/2 each) evaluated
#      once at the centre-differential input speed, so the per-set motor data keep their meaning.
#    - Engine braking and driveline inertias are not modelled; brakes stay per wheel (brake_model).
# ----------------------------------------------------------------


def engine_torque(Tm_request, omega_C, params):
    # Total driving torque at the centre differential input, in wheel units [Nm].
    # Two sim "motors" (motor_model is per motor, with maxPower/2) at the same speed make one engine.
    tau_red = params.transmission.tau_red
    eff_red = params.transmission.eff_red
    Tw_one = motor_model(Tm_request, omega_C, params)
    # Guards on the motor_model branches that ignore the pedal (see TODO(dev) notes in motor_model.py).
    # They act only in this driveline, the Chimera path keeps the original motor_model behaviour.
    Tw_one = min(Tw_one, abs(Tm_request) * tau_red * eff_red)            # zero pedal -> zero torque
    if abs(omega_C * tau_red) >= params.motor.maxRotSpeed * math.pi / 30:
        Tw_one = 0.0                                                     # no torque at or above maxRotSpeed
    return 2.0 * Tw_one


def differential_model(T_in, omega_a, omega_b, split_a, diff, k_dif):
    # One differential: input torque T_in, output speeds omega_a, omega_b (same units as T_in).
    # Returns the output torques (T_a, T_b), with T_a + T_b = T_in.
    # split_a : static torque share of output a (0.5 for an axle, front_torque_fraction for the centre)
    # diff    : DiffData(mode = "open" | "lsd" | "locked", tbr_on, tbr_off, preload)
    d_omega = omega_a - omega_b
    if diff.mode == "open":
        dT_max = 0.0
    elif diff.mode == "locked":
        dT_max = math.inf
    elif diff.mode == "lsd":
        tbr = diff.tbr_on if T_in >= 0 else diff.tbr_off                  # Biral p.59: power-on / power-off
        eps = (tbr - 1.0) / (tbr + 1.0)                                    # Biral p.48
        dT_max = diff.preload + eps * abs(T_in)                            # Biral p.60 (preload = 0 -> p.59)
    else:
        raise ValueError("unknown differential mode '%s', use open, lsd or locked" % diff.mode)
    dT = min(k_dif * abs(d_omega), dT_max)                                 # Ale's notes p.2
    s = math.copysign(1.0, d_omega) if d_omega != 0 else 0.0
    # internal torque against the speed difference: the faster output loses dT/2, the slower gains dT/2
    T_a = split_a * T_in - s * dT / 2
    T_b = (1.0 - split_a) * T_in + s * dT / 2
    return T_a, T_b


def driveline_4wd_model(Tm_request, omega__rr, omega__rl, omega__fr, omega__fl, params):
    # Returns the driving torques at the four wheels (rr, rl, fr, fl) [Nm].
    tr = params.transmission
    f = tr.front_torque_fraction
    omega_F = (omega__fr + omega__fl) / 2                                  # Willis, Biral p.37
    omega_R = (omega__rr + omega__rl) / 2
    omega_C = f * omega_F + (1.0 - f) * omega_R                            # centre input speed (power balance)
    T_C = engine_torque(Tm_request, omega_C, params)
    T_F, T_R = differential_model(T_C, omega_F, omega_R, f, tr.diff_centre, tr.k_dif)
    T_fr, T_fl = differential_model(T_F, omega__fr, omega__fl, 0.5, tr.diff_front, tr.k_dif)
    T_rr, T_rl = differential_model(T_R, omega__rr, omega__rl, 0.5, tr.diff_rear, tr.k_dif)
    return T_rr, T_rl, T_fr, T_fl
