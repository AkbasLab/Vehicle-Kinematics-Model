import warnings
warnings.simplefilter('ignore')

import scenarioxp as sxp
import sumo

import traci
import pandas as pd
import numpy as np
from typing import Callable


class DriveableAreaTest:
    def __init__(self):
        self._rng = np.random.RandomState(seed=444)
        traci_client = sumo.DriveableAreaClient()

        self._manager = sxp.ScenarioManager(pd.read_csv("params.csv"))
        self._scenario = sumo.DriveableAreaScenario
        self._tsc = lambda s : s["hhh"] > 0

        params = pd.Series({
            "dut_speed" : 30,
            "dut_dist" : 100,
            "foe_speed" : 25,
            "foe_dist" : 70,
        })
        self.scenario(params)

        traci.close()
        return

    @property
    def manager(self) -> sxp.ScenarioManager:
        return self._manager
    
    @property
    def tsc(self) -> Callable[[pd.Series], bool]:
        return self._tsc

    @property
    def scenario(self) -> sumo.DriveableAreaScenario:
        return self._scenario

    def random_seed(self):
        return self._rng.randint(2**32-1)
    
 


if __name__ == "__main__":
    DriveableAreaTest()