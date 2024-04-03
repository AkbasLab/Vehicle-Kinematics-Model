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
        self._scenario = sumo.CutInScenario
        self._tsc = lambda s : s["hhh"] > 0

        params = pd.Series({
            "s0.A" : 5,
            "s0.B" : 7,
            "s0.C" : 8.5,
            "dist.BA" : 20,
            "dist.CA" : 0,
            "dist.PA" : 20,
            "dist.P0" : 0,
            "lane_change_dur" : 5
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