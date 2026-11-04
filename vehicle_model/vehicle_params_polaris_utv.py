import math
from vehicle_model.vehicle_params import (VehicleParams, VehicleData, ChassisData, TransmissionData, DiffData,
                                          SteeringSystemData, BrakingSystemData, MotorData, AccumulatorData,
                                          WheelData, SuspensionData)

# ----------------------------------------------------------------
## Function purpose: parameter set "polaris_utv" (off-road UTV, rock-crawler candidate) for my_PyDrivingSim.
#
#  NOT IN SLIDES. External, openly licensed source: Project Chrono (BSD-3), "Polaris" UTV model,
#  github.com/projectchrono/chrono, data/vehicle/Polaris/*.json and Polaris_Pac02Tire.tir.
#  Tags: [C] value copied from Chrono, [D] derived from Chrono values (see rock_crawler_params/README.md),
#        [S] kept from the Chimera Evoluzione set (actuator/numerical constant, no Chrono equivalent),
#        [A] assumption, TODO(dev) to confirm.
# ----------------------------------------------------------------

G = 9.81

# Tyre/wheel (Polaris_Pac02Tire.tir, Polaris_Wheel.json, Polaris_Pac02Tire.json)
R0_TYRE      = 0.3683                    # [m]   [C] UNLOADED_RADIUS
CZ_TYRE      = 82715.55118               # [N/m] [C] VERTICAL_STIFFNESS
WHEEL_MASS   = 18.8 + 15.1               # [kg]  [C] rim + tyre mass
WHEEL_IWA    = 0.113 + 1.34              # [kg*m^2] [C] rim + tyre spin inertia (driveline inertia not included)


def crawler_wheel(R):
    return WheelData(R = R,                                   # [m] [D] loaded radius R0 - Fz_static/Cz
                     width = 0.2794,                          # [m] [C] WIDTH
                     mass = WHEEL_MASS,
                     iwd = WHEEL_MASS / 12 * (3 * R ** 2 + 0.2794 ** 2),
                     iwa = WHEEL_IWA,
                     static_camber = 0.0)                     # [deg] [A] not given in Chrono data


def PolarisUtvVehicleParams():
    return VehicleParams(
        vehile = VehicleData(
            Lf = 1.5515,                  # [m] [D] CoM incl. unsprung masses, axles at x = 0 / -2.71526
            Lr = 1.1638,                  # [m] [D]
            L = 2.7153,                   # [m] [D] wheelbase
            hGs = 0.625,                  # [m] [D] chassis CoM z = 0.767 in chassis frame, ground at z = 0.073
            Wf = 1.232,                   # [m] [D] 2 * spindle y = 0.616
            Wr = 1.232,                   # [m] [D]
            lx = 0.015,                   # [m] [S]
            ly = 0.015,                   # [m] [S]
            m = 1360.1,                   # [kg] [D] chassis 1105.5 + suspension links + wheels + tyres
            i_xx = 400,                   # [kg*m^2] [D] chassis 275.7 + unsprung (point masses)
            i_yy = 1387,                  # [kg*m^2] [D] chassis 853.4 + unsprung
            i_zz = 1803,                  # [kg*m^2] [D] chassis 1200 + unsprung
            i_xz = 0,                     # [kg*m^2] [C] products of inertia = 0
            CAx = 0.0,                    # [N*s^2/m^2] [A] TODO(dev): no aero data; drag < 150 N below 7 m/s
            CAzf = 0.0,                   # [N*s^2/m^2] [A] no downforce
            CAzr = 0.0,                   # [N*s^2/m^2] [A]
            Vlow_long = 4,                # [m/s] [S] TODO(dev): crawler speeds are mostly below this threshold
            Vlow_lat = 4,                 # [m/s] [S]
            n_substeps = 2),            # [-] [A] dt = 0.50 ms: wheel/tyre slip loop stable with explicit Euler (open-loop runs)
        chassis = ChassisData(is_xx = 275.6945599, is_yy = 853.4464526, is_zz = 1200, is_xz = 0),   # [C] Polaris_Chassis.json
        transmission = TransmissionData(
            tau_red = 1 / (0.267 * 0.25), # [-] [D] fixed 1st gear 0.267 x conical gear 0.25 (low range only)
            eff_red = 0.93,               # [-] [S]
            front_torque_fraction = 0.5,  # [-] [C] Polaris_DrivelineSimple.json "Front Torque Fraction"
            drive_layout = "driveline_4wd",
            diff_front = DiffData(mode = "lsd", tbr_on = 2.0, tbr_off = 2.0),   # [C] "Front Differential Max Bias": 2.0
            diff_rear = DiffData(mode = "lsd", tbr_on = 2.0, tbr_off = 2.0),    # [C] "Rear Differential Max Bias": 2.0
            diff_centre = DiffData(mode = "open"),   # [C] SimpleDriveline: fixed front torque fraction, no centre coupling
            k_dif = 700),                 # [Nm*s/rad] [A] TODO(dev): about 0.5*iwa/dt (dt = 1 ms), Euler-stable below 2*iwa/dt
        steering_system = SteeringSystemData(tau_D = 10, tau_H = 0.03),   # [S] TODO(dev): steering ratio not in Chrono data
        braking_system = BrakingSystemData(
            max_brake_torque_front = 2000,  # [Nm] [C] Polaris_BrakeShafts.json "Maximum Torque" (per wheel)
            max_brake_torque_rear = 2000,   # [Nm] [C]
            brakeRatio = 0.5,               # [-] [A]
            totBrakeTorque = 4000),         # [Nm] [D] per-wheel request = totBrakeTorque * brakeRatio = 2000
        motor = MotorData(
            maxTorque = 397.0 / 2,          # [Nm] [D] engine map peak 397 Nm @ 1500 rpm, split on the 2 sim "motors"
            speedForTorqueCut = 2600,       # [rpm] [D] last map point before 2700 rpm
            maxRotSpeed = 2700),            # [rpm] [C] "Maximal Engine Speed RPM"
        accumulator = AccumulatorData(maxPower = 77.7),   # [kW] [D] max(T*omega) of the full-throttle map
        front_wheel = crawler_wheel(R0_TYRE - 2859.2 / CZ_TYRE),   # front static load 2859 N
        rear_wheel = crawler_wheel(R0_TYRE - 3811.9 / CZ_TYRE),    # rear static load 3812 N
        front_suspension = SuspensionData(
            Ks = ((15175)**(-1) + (CZ_TYRE)**(-1))**(-1),   # [N/m] [D] spring slope 31510 N/m x MR^2 (0.694), in series with tyre
            h_rc = 0.0),                                    # [m] [A] TODO(dev): roll centre height not derived
        rear_suspension = SuspensionData(
            Ks = ((54899)**(-1) + (CZ_TYRE)**(-1))**(-1),   # [N/m] [D] 150000 N/m x MR^2 (0.605), in series with tyre
            h_rc = 0.0)                                     # [m] [A] TODO(dev)
    )
