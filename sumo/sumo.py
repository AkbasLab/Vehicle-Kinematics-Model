import shutil
import warnings
if shutil.which("sumo") is None:
    warnings.warn("Cannot find sumo/tools in the system path. Please verify that the lastest SUMO is installed from https://www.eclipse.org/sumo/")
import os

import traci
import traci.constants as tc

import scenarioxp as sxp
import pandas as pd
import re
import numpy as np
import matplotlib.pyplot as plt

from kinematic_model import KinematicBicycleModelScenario
import constants
import utils

__FILE_DIR__ = os.path.dirname(os.path.abspath(__file__))
__MAP_DIR__ = "%s/map" % __FILE_DIR__
__INIT_STATE_FN__ = "%s/init-state.xml" % __MAP_DIR__


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

            traci.start(cmd,port=self.config["--remote-port"])
            traci.setOrder(self.priority)
            return
        
        # Initialize every client after the first.
        traci.init(port=self.config["--remote-port"])
        traci.setOrder(self.priority)
        return    
    
class DriveableAreaClient(TraCIClient):
    def __init__(self):
        print(__MAP_DIR__)
        config = {
            "gui" : False,

            # Street network
            "--net-file" : "%s/4-way.net.xml" % __MAP_DIR__,

            # Logging
            "--error-log" : "%s/error.txt" % __MAP_DIR__,
            # "--log" : "%s/log.txt" % map_dir,

            # Quiet Mode
            "--no-warnings" : "",
            "--no-step-log" : "",

            # Traci Connection
            "--num-clients" : 1,
            "--remote-port" : 5522,
        }
        config["gui"] = constants.sumo.gui
        config["--delay"] = constants.sumo.delay
        if constants.sumo.start:
            config["--start"] = ""
        if constants.sumo.quit_on_end:
            config["--quit-on-end"] = ""
        config["--seed"] = constants.sumo.seed

        super().__init__(config)
        traci.simulation.saveState(__INIT_STATE_FN__)
        return
    

class DriveableAreaScenario(sxp.Scenario):
    def __init__(self, params : pd.Series):
        super().__init__(params)

        # Revert simulation state 
        traci.simulation.loadState(__INIT_STATE_FN__)
        
        # Constants
        self.DUT_ID = "DUT"
        self.FOE_ID = "FOE"
        self.DESTINATION_EDGE_IDS = ["WBW", "SBN", "NBN", "WBE", 
                                        "EBE", "NBS", "SBS"]

        self._add_vehicles()
        self._init_view()

        self._score = pd.Series({"hhh" : 50})

        # Run simulation
        while traci.simulation.getMinExpectedNumber() > 0:
            if traci.vehicle.getIDCount() != 2:
                break
            self._update_score()
            traci.simulationStep()
        return
    
    def _add_vehicles(self):
        """
        The DUT will make a left turn at the junction
        """
        traci.route.add("dut_route", ["EBW","NBN"])
        traci.vehicle.add(self.DUT_ID, "dut_route")
        traci.vehicle.setColor(self.DUT_ID, constants.RGBA.light_blue)

        # Warmup DUT
        dut_accel = traci.vehicle.getAccel(self.DUT_ID)
        speed = utils.kph2mps(self.params["dut_speed"])
        traci.vehicle.setAccel(self.DUT_ID, 1000)
        traci.vehicle.setMaxSpeed(self.DUT_ID, speed)
        traci.vehicle.setSpeed(self.DUT_ID, speed)


        """
        The FOE will travel straight ahead.
        """
        traci.route.add("foe_route", ["EBW","EBE"])
        traci.vehicle.add(self.FOE_ID, "foe_route")
        traci.vehicle.setColor(self.FOE_ID, constants.RGBA.rosey_red)

        # Warmup FOE
        foe_accel = traci.vehicle.getAccel(self.FOE_ID)
        speed = utils.kph2mps(self.params["foe_speed"])
        traci.vehicle.setAccel(self.FOE_ID, 1000)
        traci.vehicle.setMaxSpeed(self.FOE_ID, speed)
        traci.vehicle.setSpeed(self.FOE_ID, speed)


        # Spawn Vehicles
        traci.simulationStep()

        # Get up to speed
        traci.simulationStep()

        # Restore Accel and Move to stop line.
        traci.vehicle.setAccel(self.DUT_ID, dut_accel)
        pos = 100 - self.params["dut_dist"]
        traci.vehicle.moveTo(self.DUT_ID, "EBW_0", pos)

        traci.vehicle.setAccel(self.FOE_ID, foe_accel)
        pos = 100 - self.params["foe_dist"]
        traci.vehicle.moveTo(self.FOE_ID, "EBW_0", pos)
        return
    
    def _init_view(self):
        if not traci.hasGUI():
            return
        
        view_id = "View #0"
        traci.gui.setSchema(view_id, "real world")
        traci.gui.setZoom(view_id, 200)
        traci.gui.trackVehicle(view_id, self.DUT_ID)
        return
    
    def _update_score(self):
        """
        The model properties are based on the sumo default passenger vehicle
         which i a VW Golf MK7
        """
        x,y = traci.vehicle.getPosition(self.DUT_ID)
        params = pd.Series({
            "wheelbase.m" : constants.dut.wheelbase,
            "max_steer.rad" : utils.deg2rad(constants.dut.max_steering_angle),
            "time_window.s" : constants.kbm.time_window,
            "delta_time.s" : constants.kbm.delta_time,
            "x.m" : x,
            "y.m" : y,
            "yaw.rad" : utils.deg2rad(traci.vehicle.getAngle(self.DUT_ID)),
            "v.mps" : traci.vehicle.getSpeed(self.DUT_ID),
            "a_min.mps^2" : -6, #-traci.vehicle.getEmergencyDecel(self.DUT_ID),
            "a_max.mps^2" : 4 #traci.vehicle.getAccel(self.DUT_ID)
        })
        
        KinematicBicycleModelScenario(params)


        traci.close()
        quit()
        return
    
    
    
    @property
    def score(self) -> pd.Series:
        return self._score