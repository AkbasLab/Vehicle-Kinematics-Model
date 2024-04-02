
"""
DUT SETTINGS
"""
class dut:
    wheelbase = 2.63        # m
    max_steering_angle = 10 # degrees


"""
Colors
"""
class RGBA:
    light_blue = (12,158,236,255)
    rosey_red = (244,52,84,255)
    lime_green = (50,205,50,255)
    yellow = (255,255,0,255)

"""
Sumo client settings
"""
class sumo:
    gui = True
    delay = 100
    start = True
    quit_on_end = True
    seed = 333
    default_action_step_length = 1.0
    
    