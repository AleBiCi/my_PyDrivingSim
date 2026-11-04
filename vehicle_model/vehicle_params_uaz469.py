from vehicle_model.vehicle_params import (VehicleParams, VehicleData, ChassisData, TransmissionData,
                                          SteeringSystemData, BrakingSystemData, MotorData, AccumulatorData,
                                          WheelData, SuspensionData)

# ----------------------------------------------------------------
## Function purpose: parameter set "uaz469" (UAZ-469 jeep-like, leaf-spring solid axles, 4WD) for my_PyDrivingSim.
#
#  NOT IN SLIDES. External, openly licensed source: Project Chrono (BSD-3), github.com/projectchrono/chrono,
#  data/vehicle/uaz/ (vehicle/UAZ469_Vehicle.json and the files it references, driveline/UAZBUS_Driveline4WD.json,
#  powertrain/UAZBUS_*.json, tire/UAZBUS_Pac02Tire.tir).
#  NOTE: Chrono's UAZ469 chassis reuses the UAZBUS (van) chassis mass 2321 kg, so the total mass (2785 kg) is far
#  above a real UAZ-469 (UNVERIFIED, public specs give about 1.5-1.65 t kerb). TODO(dev): decide whether to keep it.
#  Tags: [C] value copied from Chrono, [D] derived from Chrono values (see rock_crawler_params/README.md),
#        [S] kept from the Chimera Evoluzione set (actuator/numerical constant, no Chrono equivalent),
#        [A] assumption, TODO(dev) to confirm.
# ----------------------------------------------------------------

# Tyre/wheel (UAZBUS_Pac02Tire.tir/.json, UAZBUS_Wheel.json)
R0_TYRE      = 0.379                     # [m]   [C] UNLOADED_RADIUS
CZ_TYRE      = 222954.5455               # [N/m] [C] VERTICAL_STIFFNESS
WHEEL_MASS   = 12.0 + 19.8               # [kg]  [C] rim + tyre mass
WHEEL_IWA    = 0.42 + 2.22357            # [kg*m^2] [C] rim + tyre spin inertia (driveline inertia not included)
TYRE_WIDTH   = 0.225                     # [m]   [C] WIDTH


def uaz_wheel(R):
    return WheelData(R = R,                                   # [m] [D] loaded radius R0 - Fz_static/Cz
                     width = TYRE_WIDTH,
                     mass = WHEEL_MASS,
                     iwd = WHEEL_MASS / 12 * (3 * R ** 2 + TYRE_WIDTH ** 2),
                     iwa = WHEEL_IWA,
                     static_camber = 0.0)                     # [deg] [C] "Camber Angle (deg)": 0


def Uaz469VehicleParams():
    return VehicleParams(
        vehile = VehicleData(
            Lf = 1.1888,                  # [m] [D] CoM incl. axles and wheels; axles at x = 0 / -2.38
            Lr = 1.1912,                  # [m] [D]
            L = 2.38,                     # [m] [C] rear "Suspension Location" x = -2.38
            hGs = 0.598,                  # [m] [D] chassis CoM 0.3 above axle centres + loaded radius
            Wf = 1.465,                   # [m] [D] 2 x spindle y 0.7325
            Wr = 1.465,                   # [m] [D]
            lx = 0.015,                   # [m] [S]
            ly = 0.015,                   # [m] [S]
            m = 2785.0,                   # [kg] [D] chassis 2321 + front axle 183.4 + rear axle 153.4 + 4 x (rim + tyre)
            i_xx = 888,                   # [kg*m^2] [D] chassis 785 + unsprung (parallel axis)
            i_yy = 3304,                  # [kg*m^2] [D] chassis 2612 + unsprung
            i_zz = 3487,                  # [kg*m^2] [D] chassis 2761 + unsprung
            i_xz = 0,                     # [kg*m^2] [C] products of inertia = 0
            CAx = 0.0,                    # [N*s^2/m^2] [A] TODO(dev): no aero data
            CAzf = 0.0,                   # [N*s^2/m^2] [A]
            CAzr = 0.0,                   # [N*s^2/m^2] [A]
            Vlow_long = 4,                # [m/s] [S] TODO(dev): crawling speeds are mostly below this threshold
            Vlow_lat = 4),                # [m/s] [S]
        chassis = ChassisData(is_xx = 785, is_yy = 2612, is_zz = 2761, is_xz = 0),   # [C] UAZ469_Chassis.json
        transmission = TransmissionData(
            tau_red = 1 / (0.125 * 0.236),  # [-] [D] 1st gear 0.125 x 4WD conical gear 0.236 (fixed gear)
            eff_red = 0.93,                 # [-] [S]
            front_torque_fraction = 0.5),   # [-] [A] UAZBUS_Driveline4WD.json: central differential, equal split assumed
        steering_system = SteeringSystemData(tau_D = 10, tau_H = 0.03),   # [S] TODO(dev): steering ratio not in Chrono data
        braking_system = BrakingSystemData(
            max_brake_torque_front = 3000,  # [Nm] [C] UAZBUS_BrakeSimple_Front.json (per wheel)
            max_brake_torque_rear = 3000,   # [Nm] [C] UAZBUS_BrakeSimple_Rear.json
            brakeRatio = 0.5,               # [-] [A]
            totBrakeTorque = 6000),         # [Nm] [D] per-wheel request = totBrakeTorque * brakeRatio = 3000
        motor = MotorData(
            maxTorque = 208.1 / 2,          # [Nm] [D] engine map peak 208.1 Nm @ 2348 rpm, split on the 2 sim "motors"
            speedForTorqueCut = 3908,       # [rpm] [C] last positive map point
            maxRotSpeed = 4000),            # [rpm] [C] "Maximal Engine Speed RPM"
        accumulator = AccumulatorData(maxPower = 79.6),   # [kW] [D] max(T*omega) of the full-throttle map
        front_wheel = uaz_wheel(R0_TYRE - 6837 / CZ_TYRE),   # front static load 6837 N
        rear_wheel = uaz_wheel(R0_TYRE - 6824 / CZ_TYRE),    # rear static load 6824 N
        front_suspension = SuspensionData(
            Ks = ((50580)**(-1) + (CZ_TYRE)**(-1))**(-1),   # [N/m] [D] leaf 102644 N/m x (spring y 0.514 / spindle y 0.733)^2, in series with tyre
            h_rc = 0.0),                                    # [m] [A] TODO(dev): roll centre height not derived
        rear_suspension = SuspensionData(
            Ks = ((50580)**(-1) + (CZ_TYRE)**(-1))**(-1),   # [N/m] [D] same leaf spring as the front
            h_rc = 0.0)                                     # [m] [A] TODO(dev)
    )
