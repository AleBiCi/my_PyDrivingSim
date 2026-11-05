# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT

import ctypes as ct
from datetime import datetime
from math import *

import agent.agent_interfaces_connector as agent_lib
from agent.interfaces_python_data_structs import input_data_str, output_data_str

from pydrivingsim import World, Vehicle, TrafficLight, TrafficCone, Target, SuggestedSpeedSignal, Coin, Obstacle

# Max objects in the scenario message (input_data_str.ObjID[20])
MAX_OBJS = 20
# Ego "GPS" pseudo-object, always in slot 0. The interface has no pose fields and cannot change
# (precompiled libagent_lib; see the GPS TODO in lib/interfaces_data_structs.h), so:
#   ObjX, ObjY = vehicle position in the world frame [m], ObjCourse = yaw [rad, CCW from world x],
#   ObjVel = speed module [m/s], ObjLen = ObjWidth = 0.
OBJ_ID_GPS = 5
# Pre-run map preview handshake (MapScenario, vehicle frozen until Start), basic_agent_ST hlp/hlp.h:
#   scenario msg AutomationLevel = -(page + 1) -> the agent replies with page `page` of its whole RRT* path,
#   path vertices [PREVIEW_PAGE_STRIDE * page, PREVIEW_PAGE_STRIDE * page + 22] in TrajectoryPointIX/IY
#   (consecutive pages share one vertex), ManoeuverType = PREVIEW_ACK + HLP status, no control.
#   AutomationLevel = 0 in every normal cycle. Neither field is used by the simulator otherwise.
PREVIEW_PAGE_STRIDE = 22
PREVIEW_ACK = 1000
PREVIEW_MAX_PAGES = 24      # RRTS_MAX_PATH = 512 vertices / 22 per page


c = agent_lib.AgentConnector()

# Define types
type_integer_4 = ct.c_int32 * 4
type_integer_10 = ct.c_int32 * 10
type_integer_20 = ct.c_int32 * 20
type_double_10 = ct.c_double * 10
type_double_20 = ct.c_double * 20
type_double_100 = ct.c_double * 100

class Agent():
    __metadata = {
        "dt": 0.05
    }
    def __init__(self, vehicle: Vehicle):
        c.test_lib()
        self.vehicle = vehicle

        # Simulation information
        assert self.__metadata["dt"] >= World().get_dt()
        self.sim_call_freq = self.__metadata["dt"] / World().get_dt()
        self.num_of_step = 0

        IP = [127, 0, 0, 1]  # Remote usage
        PORT = 30000  # Default port for agent_test
        log_enable = 0  # The log can be True only in local usage
        type_of_log = 0  # Different level of log

        # Function for agent_test initialization
        c.client_agent_init_num(type_integer_4(*IP), PORT, log_enable, type_of_log)

        self.scenario_msg = input_data_str()
        self.manoeuvre_msg = output_data_str()

        # Create the pointer for the function call
        self.scenario_msg_pointer = ct.pointer(self.scenario_msg)
        self.manoeuvre_msg_pointer = ct.pointer(self.manoeuvre_msg)

        self.cycle_number = 0
        self.requested_cruising_speed = 20
        self.action = (0,0)

        # Optional PathOverlay that draws the agent's TrajectoryPointIX/IY (set by the scenario)
        self.path_overlay = None
        # Page requested in a pre-run preview cycle (preview_path), None in normal cycles
        self.preview_page = None

        # Data to be filtered
        self.ALgtFild = 0
        self.YawRateFild = 0
        self.SteerWhlAg = 0

    def compute(self):
        self.num_of_step += 1
        self.__filtering(self.vehicle)
        if self.num_of_step >= self.sim_call_freq:
            self.__compute(self.vehicle)
            self.num_of_step = 0
            self.__clear_filter()


    def __filtering(self, v :Vehicle):
        s: input_data_str = self.scenario_msg
        self.ALgtFild += v.dX[3] - v.state[5] * v.state[4]
        self.YawRateFild += v.state[5]
        self.SteerWhlAg += v.state[10]

    def __clear_filter(self):
        s: input_data_str = self.scenario_msg
        self.ALgtFild = 0
        self.YawRateFild = 0
        self.SteerWhlAg = 0

    def __compute(self, v :Vehicle):
        s :input_data_str = self.scenario_msg
        m :output_data_str = self.manoeuvre_msg

        # Closing env if agent request to close
        if m.Status == 1:
            self.terminate()

        # Basic parameters
        self.cycle_number += 1
        s.CycleNumber = self.cycle_number
        s.ID = 0
        s.TimeStamp = ct.c_double(datetime.timestamp(datetime.now()))
        s.Status = 0
        s.ECUupTime = World().time

        # Vehicle parameters
        s.VehicleLen = v.vehicle.vehicle.L                          # double - lenght dimension [m]
        s.VehicleWidth = v.vehicle.vehicle.Wf                       # double - width dimension [m]
        s.LaneHeading = -v.state[2]
        #print(v.state[2])
        #print((v.state[0],v.state[1]))
        s.VLgtFild = v.state[3]
        s.ALgtFild = self.ALgtFild/self.num_of_step
        s.YawRateFild = self.YawRateFild/self.num_of_step
        s.SteerWhlAg = self.SteerWhlAg/self.num_of_step
        s.RequestedCruisingSpeed = self.requested_cruising_speed
        s.AutomationLevel = -(self.preview_page + 1) if self.preview_page is not None else 0

        # Lateral Position
        road_width = 4
        road_pos = v.state[1] - road_width/2
        if abs(road_pos) <= (road_width):
            s.LaneWidth = road_width
            s.LatOffsLineR = (s.LaneWidth - road_width/2) - v.state[1]
            s.LatOffsLineL = - road_width/2 - v.state[1]

        # Objects parameters (traffic light and obstacles)
        trafficlight = 0
        trafficlightDist = 0
        speedlimitId = 0
        # GPS pseudo-object (always in slot 0)
        s.ObjID[0] = OBJ_ID_GPS
        s.ObjX[0] = v.state[0]
        s.ObjY[0] = v.state[1]
        s.ObjCourse[0] = v.state[2]
        s.ObjVel[0] = sqrt(v.state[3]**2 + v.state[4]**2)
        s.ObjLen[0] = 0
        s.ObjWidth[0] = 0
        objId = 1
        target = 0
        for obj in World().obj_list:
            if type(obj) is TrafficLight:
                # Get the closest trafficlight in front
                if trafficlightDist == 0 or (obj.pos[0] - v.state[0] > -1.0 and obj.pos[0] - v.state[0] < trafficlightDist):
                    trafficlightDist = obj.pos[0] - v.state[0]
                    trafficlight = obj

            if type(obj) is TrafficCone:
                s.ObjID[objId] = 1
                delta_x = obj.pos[0] - v.state[0]
                delta_y = obj.pos[1] - v.state[1]
                s.ObjX[objId] = delta_x * cos(v.state[2]) + delta_y * sin(v.state[2])
                s.ObjY[objId] = - delta_x * sin(v.state[2]) + delta_y * cos(v.state[2])
                s.ObjVel[objId] = 0
                s.ObjLen[objId] = obj.size
                s.ObjWidth[objId] = obj.size
                objId = objId + 1

            if type(obj) is Coin:
                s.ObjID[objId] = 2
                delta_x = obj.pos[0] - v.state[0]
                delta_y = obj.pos[1] - v.state[1]
                s.ObjX[objId] = delta_x * cos(v.state[2]) + delta_y * sin(v.state[2])
                s.ObjY[objId] = - delta_x * sin(v.state[2]) + delta_y * cos(v.state[2])
                s.ObjVel[objId] = 0
                s.ObjLen[objId] = obj.size
                s.ObjWidth[objId] = obj.size
                objId = objId + 1

            # Off-road map objects. ObjID: 3 = target (goal), 4 = static obstacle (circle of diameter ObjLen)
            if (type(obj) is Target or type(obj) is Obstacle) and objId < MAX_OBJS:
                s.ObjID[objId] = 3 if type(obj) is Target else 4
                delta_x = obj.pos[0] - v.state[0]
                delta_y = obj.pos[1] - v.state[1]
                s.ObjX[objId] = delta_x * cos(v.state[2]) + delta_y * sin(v.state[2])
                s.ObjY[objId] = - delta_x * sin(v.state[2]) + delta_y * cos(v.state[2])
                s.ObjVel[objId] = 0
                s.ObjLen[objId] = obj.d if type(obj) is Obstacle else 0
                s.ObjWidth[objId] = obj.d if type(obj) is Obstacle else 0
                objId = objId + 1

            if type(obj) is SuggestedSpeedSignal:
                s.AdasisSpeedLimitValues[speedlimitId] = obj.vel
                s.AdasisSpeedLimitDist[speedlimitId] = obj.pos[0] - v.state[0]
                speedlimitId = speedlimitId + 1

        s.NrObjs = objId
        s.AdasisSpeedLimitNr = speedlimitId

        s.NrTrfLights = 0
        if trafficlight:
            s.NrTrfLights = 1
            s.TrfLightDist = trafficlight.pos[0] - v.state[0]
            s.TrfLightCurrState = trafficlight.state + 1  # 1 = Green, 2 = Yellow, 3 = Red, 0 = Flashing
            s.TrfLightFirstTimeToChange = trafficlight.time_phases[
                                              trafficlight.state] - trafficlight.time_past_switch
            s.TrfLightFirstNextState = divmod(trafficlight.state + 1, 3)[1] + 1
            s.TrfLightSecondTimeToChange = s.TrfLightFirstTimeToChange + trafficlight.time_phases[
                divmod(trafficlight.state + 1, 3)[1]]
            s.TrfLightSecondNextState = divmod(trafficlight.state + 2, 3)[1] + 1
            s.TrfLightThirdTimeToChange = s.TrfLightSecondTimeToChange + trafficlight.time_phases[
                divmod(trafficlight.state + 2, 3)[1]]

        # print("CS:" + str(s.TrfLightDist))
        # print("CS:" + str(s.TrfLightCurrState))
        # print("NS:(" + str(s.TrfLightFirstTimeToChange) + "," + str(s.TrfLightFirstNextState) + ")")
        # print("NNS:(" + str(s.TrfLightSecondTimeToChange) + "," + str(s.TrfLightSecondNextState) + ")")
        # print("NNNS:(" + str(s.TrfLightThirdTimeToChange) +")")

        c.client_agent_compute(self.scenario_msg_pointer, self.manoeuvre_msg_pointer)

        # print("ID = " + str(m.ID))
        # print("Version = " + str(m.Version))
        # print("CycleNumber = " + str(m.CycleNumber))
        # print("ECUupTime = " + str(m.ECUupTime))
        # print("Status = " + str(m.Status))
        # print("CycleNumber = " + str(m.CycleNumber))
        # print("RequestedAcc = " + str(m.RequestedAcc))

        #self.action = (0.01, 0.01)
        self.action = (m.RequestedAcc,m.RequestedSteerWhlAg)

        # Trajectory points are in the vehicle frame: draw them in the world frame
        if self.path_overlay is not None and self.preview_page is None:
            n = max(0, min(m.NTrajectoryPoints, 23))
            self.path_overlay.set_points([(v.state[0] + m.TrajectoryPointIX[i] * cos(v.state[2]) - m.TrajectoryPointIY[i] * sin(v.state[2]),
                                           v.state[1] + m.TrajectoryPointIX[i] * sin(v.state[2]) + m.TrajectoryPointIY[i] * cos(v.state[2]))
                                          for i in range(n)])

    def preview_path(self):
        """Pre-run preview: ask the agent (vehicle frozen) for its whole RRT* path, page by page.
        Returns (points [(x, y)] world frame [m], status): status = HLP status of the agent
        (0 = RRTS_OK, see basic_agent_ST hlp/hlp.h, rrt_star.h) or None if the agent did not answer
        with PREVIEW_ACK (old agent binary: points = its first trajectory window only)."""
        v = self.vehicle
        m: output_data_str = self.manoeuvre_msg
        points = []
        status = None
        for page in range(PREVIEW_MAX_PAGES):
            self.preview_page = page
            # one cycle with the current (frozen) state; num_of_step = 1 for the filtered averages
            self.__clear_filter()
            self.__filtering(v)
            self.num_of_step = 1
            self.__compute(v)
            n = max(0, min(m.NTrajectoryPoints, 23))
            pts = [(v.state[0] + m.TrajectoryPointIX[i] * cos(v.state[2]) - m.TrajectoryPointIY[i] * sin(v.state[2]),
                    v.state[1] + m.TrajectoryPointIX[i] * sin(v.state[2]) + m.TrajectoryPointIY[i] * cos(v.state[2]))
                   for i in range(n)]
            points.extend(pts if page == 0 else pts[1:])
            if m.ManoeuverType < PREVIEW_ACK - 100 or m.ManoeuverType > PREVIEW_ACK + 100:
                break       # agent without the preview handshake
            status = m.ManoeuverType - PREVIEW_ACK
            if n < 23:
                break
        self.preview_page = None
        self.num_of_step = 0
        self.__clear_filter()
        self.action = (0, 0)
        return points, status

    def terminate(self):
        World().loop = 0

        self.cycle_number += 1
        self.scenario_msg.CycleNumber = self.cycle_number
        self.scenario_msg.TimeStamp = ct.c_double(datetime.timestamp(datetime.now()))
        self.scenario_msg.ECUupTime = World().time;
        self.scenario_msg.Status = 1

        c.client_agent_compute(self.scenario_msg_pointer, self.manoeuvre_msg_pointer)
        c.client_agent_close()

    def get_action(self):
        return self.action
