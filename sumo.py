import shutil
import warnings
if shutil.which("sumo") is None:
    warnings.warn("Cannot find sumo/tools in the system path. Please verify that the lastest SUMO is installed from https://www.eclipse.org/sumo/")
import os

import traci
import traci.constants as tc

import scenarioxp as sxp
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import constants
import utils

__FILE_DIR__ = os.path.dirname(os.path.abspath(__file__))
__MAP_DIR__ = "%s/map" % __FILE_DIR__
__INIT_STATE_FN__ = "%s/init-state.xml" % __MAP_DIR__
__LOAD_CMD_FN__ = "%s/load-cmd.pkl" % __MAP_DIR__


class TraCIClient:
    def __init__(self, config : dict, priority : int = 1):
        """
        Barebones TraCI client.

        --- Parameters ---
        priority : int
            Priority of clients. MUST BE UNIQUE
        config : dict
            SUMO arguments stored as a python dictionary.
        """
        
        self._config = config
        self._priority = priority
        

        self.connect()
        return

    @property
    def priority(self) -> int:
        """
        Priority of TraCI client.
        """
        return self._priority

    @property
    def config(self) -> dict:
        """
        SUMO arguments stored as a python dictionary.
        """
        return self._config

    def run_to_end(self):
        """
        Runs the client until the end.
        """
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            # more traci commands
        return

    def close(self):
        """
        Closes the client.
        """
        traci.close()
        return


    def connect(self):
        """
        Start or initialize the TraCI connection.
        """
        warnings.simplefilter("ignore", ResourceWarning)
        # Start the traci server with the first client
        if self.priority == 1:
            cmd = []

            for key, val in self.config.items():
                if key == "gui":
                    sumo = "sumo"
                    if val: sumo +="-gui"
                    cmd.append(sumo)
                    continue
                
                if key == "--remote-port":
                    continue

                cmd.append(key)
                if val != "":
                    cmd.append(str(val))
                continue
            
            utils.save(cmd, __LOAD_CMD_FN__)
            traci.start(cmd,port=self.config["--remote-port"])
            traci.setOrder(self.priority)
            return
        
        # Initialize every client after the first.
        traci.init(port=self.config["--remote-port"])
        traci.setOrder(self.priority)
        return    
    
class DriveableAreaClient(TraCIClient):
    def __init__(self):
        """
        @param lanechange_duration
            Duration of lane change in seconds.
        """
        print(__MAP_DIR__)
        config = {
            "gui" : False,

            # Street network
            "--net-file" : "%s/3-roads.net.xml" % __MAP_DIR__,

            # Logging
            "--error-log" : "%s/error.txt" % __MAP_DIR__,
            # "--log" : "%s/log.txt" % map_dir,

            # Quiet Mode
            "--no-warnings" : "",
            "--no-step-log" : "",

            # Traci Connection
            "--num-clients" : 1,
            "--remote-port" : 5522,

            # Change in parameters not here
            "--lanechange.duration" : 1 

        }
        config["gui"] = constants.sumo.gui
        config["--delay"] = constants.sumo.delay
        if constants.sumo.start:
            config["--start"] = ""
        if constants.sumo.quit_on_end:
            config["--quit-on-end"] = ""
        config["--seed"] = constants.sumo.seed
        config["--default.action-step-length"] = constants.sumo.default_action_step_length

        super().__init__(config)
        traci.simulation.saveState(__INIT_STATE_FN__)
        return

class DriveableAreaScenario(sxp.Scenario):
    def __init__(self, 
            params : pd.Series, 
            b_enabled : bool = False, 
            c_enabled : bool = False, 
            p_enabled : bool = False,
            lane_change_rel_c : int = 0,
            lane_a : int = 0,
            lane_b : int = 0,
            lane_c : int = 0,
            rid : str = "triple"
        ):
        super().__init__(params)
        

        # Update Lane change time
        cmd : list[str] = utils.load(__LOAD_CMD_FN__)[1:]
        i = cmd.index("--lanechange.duration")
        cmd[i+1] = str(params["lane_change_dur"])

        # Reload simulation
        traci.load(cmd)

        # Constants
        self.A = "A"
        self.DUT = self.A
        self.B = "B"
        self.C = "C"
        self.P = "P"
    
        self._score = pd.Series({"hhh" : 50})

        self.lane_change_rel_c = lane_change_rel_c
        self.lane_a = lane_a
        self.lane_b = lane_b
        self.lane_c = lane_c
        self.b_enabled = b_enabled
        self.c_enabled = c_enabled
        self.p_enabled = p_enabled
        self.rid = rid
        
        if self.rid == "triple":
            self.route = ["warmup", "triple"]
            self.start_eid = "triple"
            self.dist_a = 100
        elif self.rid == "double":
            self.route = ["warmup", "double"]
            self.start_eid = "double"
            self.dist_a = 100
        elif self.rid == "ped":
            self.route = ["warmup", "pedE"]
            self.start_eid = "pedW"
            self.dist_a = 260.5 - params["dist.PA"]
        
        self._add_vehicles()
        self._init_view()

        # Run simulation
        while traci.simulation.getMinExpectedNumber() > 0:
            # if traci.vehicle.getIDCount() != 2:
            #     break
            # # self._update_score()
            traci.simulationStep()
        return
    
    def _add_vehicles(self):
        
        """
        Add and Warmup DUT
        """
        traci.route.add(self.rid, self.route)
        traci.vehicle.add(self.A, self.rid, departLane=self.lane_a)
        traci.vehicle.setColor(self.A, constants.RGBA.light_blue)

        accel_a = traci.vehicle.getAccel(self.A)
        speed = utils.kph2mps(self.params["s0.A"])
        traci.vehicle.setAccel(self.A, 1000)
        traci.vehicle.setSpeed(self.A, speed)
        traci.vehicle.setLaneChangeMode(self.A, 0) # No lane change

        if self.b_enabled:
            """
            Add and warmup B
            """
            traci.vehicle.add(self.B, self.rid)
            traci.vehicle.setColor(self.B, constants.RGBA.rosey_red)

            accel_b = traci.vehicle.getAccel(self.B)
            speed = utils.kph2mps(self.params["s0.B"])
            traci.vehicle.setAccel(self.B, 1000)
            traci.vehicle.setSpeed(self.B, speed)

        if self.c_enabled:
            """
            Add and warmup C
            """
            traci.vehicle.add(self.C, self.rid)
            traci.vehicle.setColor(self.C, constants.RGBA.lime_green)

            accel_c = traci.vehicle.getAccel(self.C)
            speed = utils.kph2mps(self.params["s0.C"])
            traci.vehicle.setAccel(self.C, 1000)
            traci.vehicle.setSpeed(self.C, speed)
            traci.vehicle.setLaneChangeMode(self.C, 0) # No Lane Change


        

        # Spawn Vehicles
        traci.simulationStep()

        

        
        """
            Teleport and move Vehicles.
            Do it twice to cancel the lane-change animation delay.
        """
        for i in range(2):
            #  Vehicle A
            traci.vehicle.setAccel(self.A, accel_a)
            traci.vehicle.moveTo(
                self.A, 
                "%s_%d" % (self.start_eid, self.lane_a), 
                self.dist_a
            )
            traci.vehicle.setSpeed(self.A,-1) # Give control back to sumo

            if self.b_enabled:
                # Vehicle B
                traci.vehicle.setAccel(self.B, accel_b)
                traci.vehicle.moveTo(
                    self.B, 
                    "%s_%d" % (self.start_eid, self.lane_b), 
                    self.dist_a + self.params["dist.BA"]
                )
                traci.vehicle.setSpeed(self.B, -1) # Give control back to sumo
            
            if self.c_enabled:
                # Vehicle C
                traci.vehicle.setAccel(self.C, accel_c)
                traci.vehicle.moveTo(
                    self.C, 
                    "%s_%d" % (self.start_eid, self.lane_c), 
                    self.dist_a + self.params["dist.CA"]
                )
                traci.vehicle.setSpeed(self.C, -1) # Give control back to sumo


            # Get up to speed
            if i == 0:
                traci.simulationStep()

        if self.c_enabled:
            traci.vehicle.changeLaneRelative(
                self.C,
                self.lane_change_rel_c,
                3
            )

        if self.p_enabled:
            """
            Add Pedestrian
            """
            cross_walk_len = 12.8
            traci.person.add(self.P, ":J5_c0", pos= self.params["dist.P0"])
            traci.person.appendWalkingStage(self.P, [":J5_c0"], cross_walk_len)
            traci.person.setColor(self.P, constants.RGBA.yellow)
        return

    
    
    def _init_view(self):
        if not traci.hasGUI():
            return
        
        view_id = "View #0"
        traci.gui.setSchema(view_id, "real world")
        traci.gui.setZoom(view_id, 400)
        traci.gui.trackVehicle(view_id, self.A)
        return
        

    # def _update_score(self):
    #     """
    #     The model properties are based on the sumo default passenger vehicle
    #      which i a VW Golf MK7
    #     """
    #     x,y = traci.vehicle.getPosition(self.DUT_ID)
    #     params = pd.Series({
    #         "wheelbase.m" : constants.dut.wheelbase,
    #         "max_steer.rad" : utils.deg2rad(constants.dut.max_steering_angle),
    #         "time_window.s" : constants.kbm.time_window,
    #         "delta_time.s" : constants.kbm.delta_time,
    #         "x.m" : x,
    #         "y.m" : y,
    #         "yaw.rad" : utils.deg2rad(traci.vehicle.getAngle(self.DUT_ID)),
    #         "v.mps" : traci.vehicle.getSpeed(self.DUT_ID),
    #         "a_min.mps^2" : -6, #-traci.vehicle.getEmergencyDecel(self.DUT_ID),
    #         "a_max.mps^2" : 4 #traci.vehicle.getAccel(self.DUT_ID)
    #     })
        
    #     KinematicBicycleModelScenario(params)


    #     traci.close()
    #     quit()
        # return
    
    @property
    def score(self) -> pd.Series:
        return self._score

    

class CutInScenario(DriveableAreaScenario):
    def __init__(self, params):
        super().__init__(
            params,
            b_enabled = True,
            c_enabled = True,
            p_enabled = False,
            rid = "double",
            lane_change_rel_c = -1,
            lane_a = 0,
            lane_b = 0,
            lane_c = 1
        )
        return
    
class TwoLaneTrafficScenario(DriveableAreaScenario):
    def __init__(self, params):
        super().__init__(
            params,
            b_enabled = True,
            c_enabled = True,
            p_enabled = False,
            rid = "double",
            lane_change_rel_c = 0,
            lane_a = 0,
            lane_b = 0,
            lane_c = 1
        )

class NoTrafficScenario(DriveableAreaScenario):
    def __init__(self, params):
        super().__init__(
            params,
            b_enabled = False,
            c_enabled = False,
            p_enabled = False,
            rid = "double",
            lane_change_rel_c = 0,
            lane_a = 0,
            lane_b = 0,
            lane_c = 0
        )

class ThreeLaneTrafficScenario(DriveableAreaScenario):
    def __init__(self, params):
        super().__init__(
            params,
            b_enabled = True,
            c_enabled = True,
            p_enabled = False,
            rid = "triple",
            lane_change_rel_c = 0,
            lane_a = 1,
            lane_b = 0,
            lane_c = 2
        )

class PedestrianCrossingScenario(DriveableAreaScenario):
    def __init__(self, params):
        super().__init__(
            params,
            b_enabled = False,
            c_enabled = False,
            p_enabled = True,
            rid = "ped",
            lane_change_rel_c = 0,
            lane_a = 1,
            lane_b = 0,
            lane_c = 0
        )