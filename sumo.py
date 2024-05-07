import shutil
import warnings
if shutil.which("sumo") is None:
    warnings.warn("Cannot find sumo/tools in the system path. Please verify that the lastest SUMO is installed from https://www.eclipse.org/sumo/")
import os
import time

import traci
import traci.constants as tc

import scenarioxp as sxp
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import constants
import utils

import shapely.geometry




from driveable_area import DriveableAreaEstimator

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
            "--log" : "%s/log.txt" % __MAP_DIR__,

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
        config["--step-length"] = constants.sumo.step_length

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
        try:
            params["lane_change_dur"]
            i = cmd.index("--lanechange.duration")
            cmd[i+1] = str(params["lane_change_dur"])
        except KeyError:
            pass

        # Reload simulation
        traci.load(cmd)

        # Constants
        self.A = "A"
        self.DUT = self.A
        self.B = "B"
        self.C = "C"
        self.P = "P"
    
        

        self.lane_change_rel_c = lane_change_rel_c
        self.lane_a = lane_a
        self.lane_b = lane_b
        self.lane_c = lane_c
        self.b_enabled = b_enabled
        self.c_enabled = c_enabled
        self.p_enabled = p_enabled
        self.rid = rid
        self.polygon_counter = 0

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

        self._score = self._init_score()

        self._add_vehicles()
        self._init_view()

        self.decel = traci.vehicle.getDecel(self.A)
        self.emergency_decel = traci.vehicle.getEmergencyDecel(self.A)
        
        self.dut_traj_df = self._predict_dut_trajectory_polygons()
        self._add_polygons_to_sumo(self.dut_traj_df, constants.RGBA.light_blue)

        traci.simulation.saveState(__INIT_STATE_FN__)

        self.veh_path_df = self._find_vehicle_paths()
        self._calc_trajectory_probability()
        self._locate_overlapping_paths()
        self._calc_entropy()
        self.entropy = self._sum_entropy_fast()        
        self.traj_entropy_breakdown_df = self._breakdown_trajectory_entropy()

        self._display_all_paths()

        self._score["entropy.A"] = self.entropy
        
        if constants.config.scenario == constants.scenario.cut_in:
            self._score["pdf(traj.C)"] = self.veh_path_df[
                self.veh_path_df["actor"] == self.C
            ]["traj_probability"].iloc[0]

        # print(self.score)
        return
    

    def _update_score(self):
        """
        Collision
        """
        colliding_vehs = traci.simulation.getCollidingVehiclesIDList()
        if self.A in colliding_vehs:
            self._score["collision.A"] = 1
            accel = 0
            speed = 0
            pos_a = (-9999,-9999)
        else:
            accel = traci.vehicle.getAcceleration(self.A)
            pos_a = traci.vehicle.getPosition(self.A)
            speed = traci.vehicle.getSpeed(self.A)
        speed = max(0.001,speed) # Speed should be non-zero for ttc calculation

        """
        Braking force of vehicle A
        """
        if accel < 0:
            decel = -accel
            decel_normal = np.round(
                min(decel / self.decel, 2.0),
                decimals = 5
            )
            self._score["max(decel.normal.A)"] = max(
                self._score["max(decel.normal.A)"],
                decel_normal
            )

            decel_mps = np.round(
                min(decel, self.emergency_decel),
                decimals = 5
            )
            self._score["max(decel.mps.A)"] = max(
                self._score["max(decel.mps.A)"],
                decel_mps
            )

       
        

        """
        Distance to Collision
        Time to Collision
        """
        if self.b_enabled:
            if self.B in colliding_vehs:
                self._score["min(dtc.m.AB)"] = 0
                self._score["min(ttc.s.AB)"] = 0
            else:
                pos_b = traci.vehicle.getPosition(self.B)

                dtc = utils.distance_to(*pos_a, *pos_b)
                self._score["min(dtc.m.AB)"] = min(
                    self._score["min(dtc.m.AB)"],
                    dtc
                )

                ttc = dtc/speed
                self._score["min(ttc.s.AB)"] = min(
                    self._score["min(ttc.s.AB)"],
                    ttc
                )


        if self.c_enabled:
            if self.C in colliding_vehs:
                self._score["min(dtc.m.AC)"] = 0
                self._score["min(ttc.s.AC)"] = 0
            else:
                pos_c = traci.vehicle.getPosition(self.C)

                dtc = utils.distance_to(*pos_a, *pos_c)
                self._score["min(dtc.m.AC)"] = min(
                    self._score["min(dtc.m.AC)"],
                    dtc
                )

                ttc = dtc/speed
                self._score["min(ttc.s.AC)"] = min(
                    self._score["min(ttc.s.AC)"],
                    ttc
                )

        if self.p_enabled:
            pos_p = traci.person.getPosition(self.P)

            dtc = utils.distance_to(*pos_a, *pos_p)
            self._score["min(dtc.m.AP)"] = min(
                self._score["min(dtc.m.AP)"],
                dtc
            )

            ttc = dtc/speed
            self._score["min(ttc.s.AP)"] = min(
                self._score["min(ttc.s.AP)"],
                ttc
            )
        return
    
    def _init_score(self) -> pd.Series:
        data = {
            "entropy.A" : -1,
            "max(decel.normal.A)" : 0,
            "max(decel.mps.A)" : 0,
            "collision.A" : 0,
        }

        if self.b_enabled:
            data["min(ttc.s.AB)"] = 9999
            data["min(dtc.m.AB)"] = 9999
        if self.c_enabled:
            data["min(ttc.s.AC)"] = 9999
            data["min(dtc.m.AC)"] = 9999
            if constants.config.scenario == constants.scenario.cut_in:
                data["pdf(traj.C)"] = 9999
        if self.p_enabled:
            data["min(ttc.s.AP)"] = 9999
            data["min(dtc.m.AP)"] = 9999

        return pd.Series(data).sort_index()
    
    def _driveable_area(self):
        """
        Non DUT paths
        """
        foe_paths = self.veh_path_df[self.veh_path_df["actor"] != self.A]

        """
        Split the driveable Area
        """
        data = {"polygon" : []}
        for poly_dut in self.dut_traj_df["polygon"]:
            poly : shapely.geometry.Polygon = poly_dut
            # Get origin
            origin = shapely.geometry.Point(poly.exterior.coords[0])

            # Split up the polygon by path
            for poly_foe in foe_paths["polygon"]:
                poly = poly.difference(poly_foe)
                continue
            
            # If it's a multipolygon, we must find the closest 
            if isinstance(poly, shapely.geometry.multipolygon.MultiPolygon):
                poly = utils.closest_polygon(origin, poly)
            
            data["polygon"].append(poly)
            continue
        df = pd.DataFrame(data)
        
        
        # 
        self._add_polygons_to_sumo(df, constants.RGBA.yellow, layer=8)
        return 
    

    def _display_all_paths(self):
        if not (constants.sumo.gui and constants.sumo.show_path_history):
            return

        traci.simulation.loadState(__INIT_STATE_FN__)

        color_map = {
            "A" : constants.RGBA.aquamarine,
            "B" : constants.RGBA.rosey_red,
            "C" : constants.RGBA.lime_green,
            "P" : constants.RGBA.yellow
        }
        for i in range(len(self.veh_path_df.index)):
            df = self.veh_path_df[self.veh_path_df.index == i]
            color = color_map[df.iloc[0]["actor"]]
            self._add_polygons_to_sumo( df, color, layer = 7)
            continue    
        
        if constants.sumo.show_path_time < 0:
            input("pause")
        else:
            time.sleep(constants.sumo.show_path_time)
        
        return


    
    def _breakdown_trajectory_entropy(self) -> pd.DataFrame:
        # print(self.dut_traj_df)
        """
        Breakdown trajectory entropy into a neat table
        """
        data = {
            "ID" : self.dut_traj_df["ID"],
            self.A : self.dut_traj_df["entropy"]
        }
        # Initilize other actors as 0
        foe_df = self.veh_path_df[self.veh_path_df["actor"] != self.A]
        for actor in foe_df["actor"]:
            data[actor] = 0.
        
        df = pd.DataFrame(data)

        # Update dataframe for other actors
        for i in range(len(foe_df.index)):
            s = foe_df.iloc[i]
            df[s["actor"]] = df["ID"].isin(s["crosses_traj_ids"]) \
                * s["entropy"]
            continue

        # print(df)
        return df

    def _sum_entropy_fast(self) -> float:
        """
        Find the entropy of A
        """
        entropy_foe = self.veh_path_df[
            (self.veh_path_df["actor"] != self.A)
        ]["entropy_all"].sum()
        
        entropy_a = self.dut_traj_df["entropy"].sum()

        entropy = entropy_a + entropy_foe
        return entropy

    def _calc_entropy(self):
        # print(self.veh_path_df)
        """
        Find the entropy of simulated paths.
        """
        self.veh_path_df["entropy"] = self.veh_path_df["traj_probability"]\
            .apply(utils.entropy)
        self.veh_path_df["entropy_all"] = \
            self.veh_path_df["crosses_traj_ids"].apply(len) \
            * self.veh_path_df["entropy"]
        # print(self.veh_path_df)
        
        """
        Find the entropy for A trajectories
        """
        self.dut_traj_df["entropy"] = self.dut_traj_df["traj_probability"]\
            .apply(utils.entropy)
        # print(self.dut_traj_df)
        return
    
    def _locate_overlapping_paths(self):
        overlapping_trajectory_ids = []
        for path_polygon in self.veh_path_df["polygon"]:
            flags = self.dut_traj_df["polygon"].apply(
                lambda traj_polygon : traj_polygon.intersects(path_polygon) 
            )
            tid = self.dut_traj_df[flags]["ID"].tolist()
            overlapping_trajectory_ids.append(tid)
        self.veh_path_df["crosses_traj_ids"] = overlapping_trajectory_ids
        return


    def _calc_trajectory_probability(self):
        df = self.veh_path_df
        
        """
        To predict the trajectory likelyhood we use a guassian distribution
        with mu = 0, sigma = 1.
        We will estimate it based on the change of angle of the vehicle.
        """
        traj_probability = []
        for i,delta_angle in enumerate(df["delta_angle.deg"]):
            # Transform into numpy array for easier manipulation
            delta_angle = np.array(delta_angle)
            
            # Subtract road curvature
            delta_angle -= constants.kinematics_model.road_curvature

            # Case: all zeros
            if np.all(delta_angle == 0.):
                x = 0.
            # Case: Angle turns
            else:
                # We only consider the parts which are turning
                delta_angle = delta_angle[delta_angle != 0]
                pred_max_steering_angle = abs(delta_angle).max()
                
                # The steering angle depends on the actor
                if df["actor"].iloc == self.P:
                    max_steering_angle = constants.pedestrian.max_steering_angle
                else:
                    max_steering_angle = constants.dut.max_steering_angle

                # Fit it between 0 and max steering angle to get a normal value
                n = pred_max_steering_angle / max_steering_angle

                # Scale between 0 and 5
                x = n*5

            # Finally find the probability
            if constants.kinematics_model.distribution == constants.distribution.gaussian:
                probability = utils.gaussian_pdf(x, mu=0, sigma=1)
            else:
                probability = 1/14
            traj_probability.append(probability)
            continue

        df["traj_probability"] = traj_probability
        return 


 


    def _find_vehicle_paths(self) -> pd.DataFrame:
        """
        Runs the simulation once to find vehicle paths.
        """
        # Dictionary to track vehicle path
        veh_paths = {}
        for vid in traci.vehicle.getIDList():
            veh_paths[vid] = []
        for pid in traci.person.getIDList():
            veh_paths[pid] = []
        self.veh_paths = veh_paths

        # Actor widths
        actor_widths = []
        for vid in traci.vehicle.getIDList():
            actor_widths.append(traci.vehicle.getWidth(vid))
        for pid in traci.person.getIDList():
            actor_widths.append(traci.person.getWidth(pid))

        # Actor Angle
        veh_angles = {}
        for vid in traci.vehicle.getIDList():
            veh_angles[vid] = []
        for pid in traci.person.getIDList():
            veh_angles[pid] = []

        

        # Run simulation
        warmup_time = traci.simulation.getTime()
        end_time = warmup_time + constants.kinematics_model.time_window
        while traci.simulation.getMinExpectedNumber() > 0:

            self._update_score()

            # Add positions and angles
            for vid in traci.vehicle.getIDList():
                veh_paths[vid].append(traci.vehicle.getPosition(vid))
                veh_angles[vid].append(traci.vehicle.getAngle(vid))
            for pid in traci.person.getIDList():
                veh_paths[pid].append(traci.person.getPosition(pid))
                veh_angles[pid].append(traci.person.getAngle(pid))
            
            # End early on a collision.
            if traci.simulation.getCollidingVehiclesNumber():
                break

            traci.simulationStep()

            # Time window complete
            if traci.simulation.getTime() >= end_time:
                break

            continue
        
        # Make at least 2 coordinate pairs
        for key,val  in veh_paths.items():
            if len(val) == 1:
                veh_paths[key].append([val[0][0]+0.01, val[0][1]])
        

        # Create the Dataframe
        veh_path_df = pd.DataFrame({
            "actor" : veh_paths.keys(),
            "width" : actor_widths,
            # "length" : actor_lengths,
            "linestring" : [shapely.geometry.LineString(val) \
                            for val in veh_paths.values()],
        })


         # Angle
        veh_path_df["angle.deg"] = [val for val in veh_angles.values()]

        # Change in Angle
        delta_angles = [[angle[i] - angle[i-1] for i in range(1,len(angle))] \
                        for angle in veh_path_df["angle.deg"]]
        veh_path_df["delta_angle.deg"] = delta_angles

        
        
        # Create Polygons
        veh_path_df["polygon"] = veh_path_df.apply(
            lambda s: utils.linestring2polygon(s["linestring"], s["width"]),
            axis = 1
        )
        
       


        

        return veh_path_df
    
    def _add_vehicles(self):
        
        """
        Add and Warmup DUT
        """
        traci.route.add(self.rid, self.route)
        traci.vehicle.add(self.A, self.rid, departLane=self.lane_a)
        traci.vehicle.setColor(self.A, constants.RGBA.light_blue)
        traci.vehicle.setMaxSpeed(self.A, constants.dut.max_speed)

        accel_a = traci.vehicle.getAccel(self.A)
        speed = self.params["s0.A"]
        traci.vehicle.setAccel(self.A, 1000)
        traci.vehicle.setSpeed(self.A, speed)
        traci.vehicle.setLaneChangeMode(self.A, 0) # No lane change

        if self.b_enabled:
            """
            Add and warmup B
            """
            traci.vehicle.add(self.B, self.rid)
            traci.vehicle.setColor(self.B, constants.RGBA.rosey_red)
            traci.vehicle.setMaxSpeed(self.B, constants.dut.max_speed)

            accel_b = traci.vehicle.getAccel(self.B)
            speed = self.params["s0.B"]
            traci.vehicle.setAccel(self.B, 1000)
            traci.vehicle.setSpeed(self.B, speed)

        if self.c_enabled:
            """
            Add and warmup C
            """
            traci.vehicle.add(self.C, self.rid)
            traci.vehicle.setColor(self.C, constants.RGBA.lime_green)
            traci.vehicle.setMaxSpeed(self.C, constants.dut.max_speed)

            accel_c = traci.vehicle.getAccel(self.C)
            speed = self.params["s0.C"]
            traci.vehicle.setAccel(self.C, 1000)
            traci.vehicle.setSpeed(self.C, speed)
            traci.vehicle.setLaneChangeMode(self.C, 0) # No Lane Change

        if self.p_enabled:
            """
            Add Pedestrian
            """
            cross_walk_len = 12.8
            traci.person.add(self.P, ":J5_c0", pos= self.params["dist.P0"])
            traci.person.appendWalkingStage(self.P, [":J5_c0"], cross_walk_len)
            traci.person.setColor(self.P, constants.RGBA.yellow)
            traci.person.setSpeed(self.P,0)

        

        # Spawn Vehicles
        traci.simulationStep()

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
            

            if self.b_enabled:
                # Vehicle B
                traci.vehicle.setAccel(self.B, accel_b)
                traci.vehicle.moveTo(
                    self.B, 
                    "%s_%d" % (self.start_eid, self.lane_b), 
                    self.dist_a + self.params["dist.BA"]
                )
                
            
            if self.c_enabled:
                # Vehicle C
                traci.vehicle.setAccel(self.C, accel_c)
                traci.vehicle.moveTo(
                    self.C, 
                    "%s_%d" % (self.start_eid, self.lane_c), 
                    self.dist_a + self.params["dist.CA"]
                )
                

            
        

            # Get up to speed
            if i == 0:
                traci.simulationStep()

                


        # Give speed control back to SUMO
        traci.vehicle.setSpeed(self.A,-1)
        if self.b_enabled:
            traci.vehicle.setSpeed(self.B, -1) 
        if self.c_enabled:
            traci.vehicle.setSpeed(self.C, -1) 
        if self.p_enabled:
            traci.person.setSpeed(self.P,-1)
            

        if self.c_enabled:
            traci.vehicle.changeLaneRelative(
                self.C,
                self.lane_change_rel_c,
                3
            )

        
        
        return

    
    
    def _init_view(self):
        if not constants.sumo.gui:
        # if not traci.hasGUI():
            return
        
        view_id = "View #0"
        traci.gui.setSchema(view_id, "real world")
        traci.gui.setZoom(view_id, 800)
        traci.gui.trackVehicle(view_id, self.A)
        return
        
    def _predict_dut_trajectory_polygons(self) -> pd.DataFrame:
        
        """
        Convert Maja's ids to steering angles
        """
        traj_ids = np.array([5,4,3,2,1.5,1,.5,0,-.5,-1.,-1.5,-2,-3,-4,-5])

        if constants.kinematics_model.distribution == constants.distribution.gaussian:
            traj_probability = utils.gaussian_pdf(traj_ids, mu = 0, sigma = 1)
        else:
            traj_probability = np.array([1/14 for _ in range(len(traj_ids))])
        
        tids = (traj_ids + 5) / 10
        delta_samples = [
            sxp.project(
                - constants.dut.max_steering_angle, 
                constants.dut.max_steering_angle, 
                tid
            ) for tid in tids
        ]        

        


        """
        Construct the estimator
        """
        x, y = traci.vehicle.getPosition(self.A)
        dae = DriveableAreaEstimator(
            a_min = -traci.vehicle.getEmergencyDecel(self.A),
            a_max = traci.vehicle.getAccel(self.A),
            n_intervals_a = constants.kinematics_model.n_intervals_a,
            delta_min = -constants.dut.max_steering_angle,
            delta_max = constants.dut.max_steering_angle,
            delta_samples = delta_samples,
            v_max = traci.vehicle.getMaxSpeed(self.A),
            lf = constants.dut.wheelbase / 2,
            lr = constants.dut.wheelbase / 2,
            time_window = constants.kinematics_model.time_window,
            dt = constants.kinematics_model.dt,
            x0 = x,
            y0 = y,
            v0 = traci.vehicle.getSpeed(self.A),
            phi0 = traci.vehicle.getAngle(self.A)-90,
            width = traci.vehicle.getWidth(self.A)
        )
        

        """
        Shorten Trajectory Polygons
        """
        # Get lane shapes
        eid = traci.vehicle.getRoadID(self.A)
        edge_polygon = self._get_edge_polygon(eid)


        # Ped specific
        if self.rid == "ped":
            ped_e_polygon = self._get_edge_polygon("pedE")
            patch_polygon = shapely.geometry.Polygon([
                [244.85,-30.03], 
                [254.21,-30.03], 
                [254.19,-36.38], 
                [245.41,-36.39], 
                [244.87,-30.06]
            ])
            edge_polygon = edge_polygon.union(patch_polygon)
            edge_polygon = edge_polygon.union(ped_e_polygon)


        # Shorten
        df = dae.trajectory_polygons
        df["polygon"] = [polygon.intersection(edge_polygon) \
         for polygon in df["polygon"]]
        df["traj_probability"] = traj_probability
        df["ID"] = traj_ids
        return df
    
    def _add_polygons_to_sumo(self, 
            df : pd.DataFrame, 
            rgba : tuple[int,int,int,int],
            layer : int = 6
        ):
        """
            Create Polygons in SUMO Optional
        """
        if not (constants.sumo.gui and constants.sumo.show_trajectories):
            return
        prefix = "poly_"
        for i in range(len(df.index)):
            s = df.iloc[i]
            shape : shapely.geometry.Polygon = s["polygon"]
            try:
                shape.boundary.coords
            except NotImplementedError:
                continue
            except ValueError:
                continue
            traci.polygon.add(
                polygonID = "%s%d" % (prefix, self.polygon_counter),
                shape = list(shape.boundary.coords),
                color = rgba,
                # fill = True,
                layer = layer,
                lineWidth = 0.1
            )
            self.polygon_counter += 1 
            continue
        return
    
    def _get_edge_polygon(self, eid : str) -> shapely.geometry.Polygon:
        """
        Get the polygon of an edge by iterating through lanes
        """
        n_lanes = traci.edge.getLaneNumber(eid)
        polygons : list[shapely.geometry.Polygon] = []
        for i_lane in range(n_lanes):
            lid = "%s_%d" % (eid, i_lane)
            # Exclude pedestrian edges
            if "pedestrian" in traci.lane.getAllowed(lid):
                continue
            shape = traci.lane.getShape(lid)
            width = traci.lane.getWidth(lid)
            linestring = shapely.geometry.LineString(shape)
            polygon = utils.linestring2polygon(linestring,width)
            polygons.append(polygon)
            continue
        
        if len(polygons) == 1:
            return polygons[0]
        
        #Join lanes
        polygon = polygons[0]
        for i in range(1,len(polygons)):
            polygon = polygon.union(polygons[i])
        return polygon

    
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