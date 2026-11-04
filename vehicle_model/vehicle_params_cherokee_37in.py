from vehicle_model.vehicle_params import (VehicleParams, VehicleData, ChassisData, TransmissionData,
                                          SteeringSystemData, BrakingSystemData, MotorData, AccumulatorData,
                                          WheelData, SuspensionData)

# ----------------------------------------------------------------
## Function purpose: parameter set "cherokee_37in" (Jeep Cherokee 1997 on 37x12.5R16.5 tyres,
#                    classic solid-axle rock-crawler base) for my_PyDrivingSim.
#
#  NOT IN SLIDES. External, openly licensed source: Project Chrono (BSD-3), github.com/projectchrono/chrono,
#  src/chrono_models/vehicle/jeep/Cherokee_*.cpp ("Vehicle Parameters taken from SAE Paper 1999-01-0121"),
#  tyre from data/vehicle/hmmwv/tire/HMMWV_Pac02Tire.tir (Chrono has no Magic Formula tyre for the Cherokee).
#  Tags: [C] value copied from Chrono, [D] derived from Chrono values (see rock_crawler_params/README.md),
#        [S] kept from the Chimera Evoluzione set (actuator/numerical constant, no Chrono equivalent),
#        [A] assumption, TODO(dev) to confirm.
# ----------------------------------------------------------------

# Tyre/wheel (HMMWV_Pac02Tire.tir/.json, Cherokee_Wheel.cpp)
R0_TYRE      = 0.4699                    # [m]   [C] UNLOADED_RADIUS
CZ_TYRE      = 1e6 * 0.4419              # [N/m] [C] VERTICAL_STIFFNESS x LCZ
WHEEL_MASS   = 11.38 + 37.6              # [kg]  [C] Cherokee rim + HMMWV tyre mass
WHEEL_IWA    = 0.9708 + 6.69             # [kg*m^2] [C] rim + tyre spin inertia (driveline inertia not included)
TYRE_WIDTH   = 0.3175                    # [m]   [C] WIDTH


def cherokee_wheel(R):
    return WheelData(R = R,                                   # [m] [D] loaded radius R0 - Fz_static/Cz
                     width = TYRE_WIDTH,
                     mass = WHEEL_MASS,
                     iwd = WHEEL_MASS / 12 * (3 * R ** 2 + TYRE_WIDTH ** 2),
                     iwa = WHEEL_IWA,
                     static_camber = 0.0)                     # [deg] [A] solid axle, no static camber given


def CherokeeVehicleParams():
    return VehicleParams(
        vehile = VehicleData(
            Lf = 1.1338,                  # [m] [D] CoM incl. axles and wheels; axles at x = 0 / -2.578
            Lr = 1.4442,                  # [m] [D]
            L = 2.578,                    # [m] [C] rear axle at x = -2.578 (Cherokee_Vehicle.cpp)
            hGs = 0.622,                  # [m] [D] chassis CoM 0.213 above axle centres + loaded radius (37 in tyres)
            Wf = 1.496,                   # [m] [D] 2 x front spindle y 0.74803
            Wr = 1.476,                   # [m] [D] 2 x rear spindle y 0.73787
            lx = 0.015,                   # [m] [S]
            ly = 0.015,                   # [m] [S]
            m = 2155.4,                   # [kg] [D] chassis 1663 + front axle 186.4 + rear axle 110.0 + 4 x (rim + tyre)
            i_xx = 815,                   # [kg*m^2] [D] chassis 653 + unsprung (parallel axis)
            i_yy = 3315,                  # [kg*m^2] [D] chassis 2498 + unsprung
            i_zz = 3649,                  # [kg*m^2] [D] chassis 2704 + unsprung
            i_xz = 0,                     # [kg*m^2] [C] only Ixy = 85 given (not used by the sim)
            CAx = 0.0,                    # [N*s^2/m^2] [A] TODO(dev): no aero data; top speed 4.4 m/s
            CAzf = 0.0,                   # [N*s^2/m^2] [A]
            CAzr = 0.0,                   # [N*s^2/m^2] [A]
            Vlow_long = 4,                # [m/s] [S] TODO(dev): crawler speeds are mostly below this threshold
            Vlow_lat = 4),                # [m/s] [S]
        chassis = ChassisData(is_xx = 653, is_yy = 2498, is_zz = 2704, is_xz = 0),   # [C] Cherokee_Chassis.cpp
        transmission = TransmissionData(
            tau_red = 11.49 / 0.2,        # [-] [D] lowest gear 1/11.49 ("torque converter and low range") / conical gear 0.2
            eff_red = 0.93,               # [-] [S]
            front_torque_fraction = 0.5), # [-] [A] Cherokee_Driveline4WD: central differential, equal split assumed
        steering_system = SteeringSystemData(tau_D = 10, tau_H = 0.03),   # [S] TODO(dev): steering ratio not in Chrono data
        braking_system = BrakingSystemData(
            max_brake_torque_front = 4000,  # [Nm] [C] Cherokee_BrakeShafts.cpp m_maxtorque (per wheel)
            max_brake_torque_rear = 4000,   # [Nm] [C]
            brakeRatio = 0.5,               # [-] [A]
            totBrakeTorque = 8000),         # [Nm] [D] per-wheel request = totBrakeTorque * brakeRatio = 4000
        motor = MotorData(
            maxTorque = 316.2 / 2,          # [Nm] [D] engine map peak 316.2 Nm @ 3500 rpm, split on the 2 sim "motors"
            speedForTorqueCut = 5000,       # [rpm] [D] last map point before 5300 rpm
            maxRotSpeed = 5300),            # [rpm] [C] Cherokee_EngineSimpleMap::GetMaxEngineSpeed
        accumulator = AccumulatorData(maxPower = 139.9),  # [kW] [D] max(T*omega) of the full-throttle map
        front_wheel = cherokee_wheel(R0_TYRE - 5923 / CZ_TYRE),   # front static load 5923 N
        rear_wheel = cherokee_wheel(R0_TYRE - 4649 / CZ_TYRE),    # rear static load 4649 N
        front_suspension = SuspensionData(
            Ks = ((9236)**(-1) + (CZ_TYRE)**(-1))**(-1),    # [N/m] [D] coil 25000 N/m x (spring y 0.455 / spindle y 0.748)^2, in series with tyre
            h_rc = 0.0),                                    # [m] [A] TODO(dev): roll centre height not derived
        rear_suspension = SuspensionData(
            Ks = ((14014)**(-1) + (CZ_TYRE)**(-1))**(-1),   # [N/m] [D] leaf 25000 N/m x (spring y 0.552 / spindle y 0.738)^2, in series with tyre
            h_rc = 0.0)                                     # [m] [A] TODO(dev)
    )
