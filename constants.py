
"""
Scenario names
"""
class scenario:
    cut_in = "cut-in"
    no_traffic = "no-traffic"
    two_lane_traffic = "2-lane-traffic"
    three_lane_traffic = "3-lane-traffic"
    pedestrian_crossing = "pedestrian"

"""
Testing Strategies
"""
class strategy:
    monte_carlo = "mc"

"""
Running configuration Options
"""
class config:
    scenario = scenario.pedestrian_crossing
    tsc = lambda s : False
    seed = 444
    strategy = strategy.monte_carlo
    n_tests = 10_000

"""
DUT SETTINGS
"""
class dut:
    wheelbase = 2.63        # m
    max_steering_angle = 10 # degrees
    max_speed = 15          # mps

"""
PEDESTRIAN SETTINGS
"""
class pedestrian:
    max_steering_angle = 90 # degrees


"""
Distribution names
"""
class distribution:
    gaussian = "gaussian standard normal"
    uniform = "uniform random"

"""
Misc Kinematics Model configuration
"""
class kinematics_model:
    n_intervals_a = 3
    time_window = 3.    # seconds
    dt = 0.1            # seconds
    road_curvature = 0. # degrees
    distribution = distribution.gaussian

"""
Colors
"""
class RGBA:
    light_blue = (12,158,236,255)
    medium_blue = (0,0,205,255)
    rosey_red = (244,52,84,255)
    lime_green = (50,205,50,255)
    red = (255,0,0,255)
    orange = (255,165,0,255)
    yellow = (255,255,0,255)
    green = (0,128,0,255)
    aquamarine = (127,255,212,255)
    

    
"""
Sumo client settings
"""
class sumo:
    gui = False
    delay = 500
    start = True
    quit_on_end = True
    show_trajectories = True
    show_path_history = False
    show_path_time = 1 # in seconds
    seed = 333
    default_action_step_length = 0.1
    step_length = 0.1

"""
Presentation Layers
"""
class presentation_layers:
    above_road = 3
    above_crosswalk = 5
    
"""
Feature and Score names 
"""
features = {
    scenario.cut_in : [
        "s0.A", "s0.B", "s0.C", "dist.BA", "dist.CA", "lane_change_dur"
    ],
    scenario.no_traffic : ["s0.A"],
    scenario.two_lane_traffic : ["s0.A", "s0.B", "s0.C", "dist.BA", "dist.CA"],
    scenario.three_lane_traffic : ["s0.A", "s0.B", "s0.C", "dist.BA", "dist.CA"],
    scenario.pedestrian_crossing : ["s0.A", "dist.PA", "dist.P0"]
}

scores = {
    scenario.cut_in : [
        "collision.A", "entropy.A", "max(decel.mps.A)", "max(decel.normal.A)", 
        "min(dtc.m.AB)", "min(dtc.m.AC)", "min(ttc.s.AB)", "min(ttc.s.AC)", 
        "pdf(traj.C)"
    ],
    scenario.no_traffic : [
        "collision.A","entropy.A", "max(decel.mps.A)", "max(decel.normal.A)"
    ],
    scenario.two_lane_traffic : [
        "collision.A", "entropy.A", "max(decel.mps.A)", "max(decel.normal.A)", 
        "min(dtc.m.AB)", "min(dtc.m.AC)", "min(ttc.s.AB)", "min(ttc.s.AC)" 
    ],
    scenario.three_lane_traffic : [
        "collision.A", "entropy.A", "max(decel.mps.A)", "max(decel.normal.A)", 
        "min(dtc.m.AB)", "min(dtc.m.AC)", "min(ttc.s.AB)", "min(ttc.s.AC)"
    ],
    scenario.pedestrian_crossing : [
        "collision.A", "entropy.A", "max(decel.mps.A)", "max(decel.normal.A)", 
        "min(dtc.m.AP)", "min(ttc.s.AP)"
    ]
}
    