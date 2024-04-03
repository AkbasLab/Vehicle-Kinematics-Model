
"""
DUT SETTINGS
"""
class dut:
    wheelbase = 2.63        # m
    max_steering_angle = 10 # degrees

"""
Misc Kinematics Model configuration
"""
class kinematics_model:
    n_intervals_a = 3
    time_window = 3. # seconds
    dt = 0.1 # seconds



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
    

    
"""
Sumo client settings
"""
class sumo:
    gui = True
    delay = 500
    start = True
    quit_on_end = True
    show_trajectories = True
    seed = 333
    default_action_step_length = 0.1
    step_length = 0.1

"""
Presentation Layers
"""
class presentation_layers:
    above_road = 3
    above_crosswalk = 5
    
    