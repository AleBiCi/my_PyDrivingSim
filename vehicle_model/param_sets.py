from vehicle_model.vehicle_params import VehicleParams
from vehicle_model.pacejka_params import PacejkaParam
from vehicle_model.vehicle_params_polaris_utv import PolarisUtvVehicleParams
from vehicle_model.pacejka_params_polaris_29x9_14 import PacejkaParamPolaris29x9_14
from vehicle_model.vehicle_params_cherokee_37in import CherokeeVehicleParams
from vehicle_model.pacejka_params_hmmwv_37x12_5 import PacejkaParamHMMWV37x12_5
from vehicle_model.vehicle_params_uaz469 import Uaz469VehicleParams
from vehicle_model.pacejka_params_uaz_225_75r16 import PacejkaParamUAZ225_75R16

# ----------------------------------------------------------------
## Function purpose: named vehicle + tyre parameter sets, selected with Vehicle(param_set = "<name>")
#
#  "chimera"       : Chimera Evoluzione + Hoosier 18x6-10 (simulator default, prof. Rosati Papini)
#  "polaris_utv"   : off-road UTV + 29x9-14 tyre        (Project Chrono, NOT IN SLIDES)
#  "cherokee_37in" : Jeep Cherokee 1997 + 37x12.5R16.5  (Project Chrono, NOT IN SLIDES)
#  "uaz469"        : UAZ-469 4WD + 225/75 R16            (Project Chrono, NOT IN SLIDES)
# ----------------------------------------------------------------

PARAM_SETS = {
    "chimera":       (VehicleParams,           PacejkaParam),
    "polaris_utv":   (PolarisUtvVehicleParams, PacejkaParamPolaris29x9_14),
    "cherokee_37in": (CherokeeVehicleParams,   PacejkaParamHMMWV37x12_5),
    "uaz469":        (Uaz469VehicleParams,     PacejkaParamUAZ225_75R16),
}


def load_param_set(name):
    if name not in PARAM_SETS:
        raise ValueError("unknown param_set '%s', choose one of %s" % (name, list(PARAM_SETS)))
    vehicle, pacejka = PARAM_SETS[name]
    return vehicle(), pacejka()
