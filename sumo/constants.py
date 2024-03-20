
"""
DUT SETTINGS
"""
class dut:
    wheelbase = 2.63        # m
    max_steering_angle = 10 # degrees

"""
Kinematic Bicycle Model Settings
"""
class kbm:
    time_window = 3 # s
    delta_time = .1 # s

"""
Colors
"""
class RGBA:
    light_blue = (12,158,236,255)
    rosey_red = (244,52,84,255)

"""
Sumo client settings
"""
class sumo:
    gui = True
    delay = 100
    start = False
    quit_on_end = True
    seed = 333
    
    