import warnings
warnings.simplefilter('ignore')

import scenarioxp as sxp
import sumo
import constants

import traci
import pandas as pd
import numpy as np
from typing import Callable


class DriveableAreaTest:
    def __init__(self):
        """
        Load Data
        """
        self._rng = np.random.RandomState( seed = constants.config.seed)
        traci_client = sumo.DriveableAreaClient()
        self._tsc = constants.config.tsc

        scenario_map = {
            constants.scenario.cut_in : sumo.CutInScenario,
            constants.scenario.no_traffic : sumo.NoTrafficScenario,
            constants.scenario.two_lane_traffic : sumo.TwoLaneTrafficScenario,
            constants.scenario.three_lane_traffic : sumo.ThreeLaneTrafficScenario,
            constants.scenario.pedestrian_crossing : sumo.PedestrianCrossingScenario
        }

        fn = "scenarios/%s.csv" % constants.config.scenario
        self._manager = sxp.ScenarioManager(pd.read_csv(fn))
        self._scenario = scenario_map[constants.config.scenario]
        
        """
        Choose Strategy
        """
        if constants.config.strategy == constants.strategy.monte_carlo:
            self.monte_carlo_strategy()
        else:
            raise NotImplementedError("Strategy %s not supported." % constants.config.strategy)

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
    
    def monte_carlo_strategy(self):
        exp = sxp.SequenceExplorer(
            strategy = sxp.SequenceExplorer.MONTE_CARLO,
            seed = self.random_seed(),
            scenario_manager = self.manager,
            scenario = self._scenario,
            target_score_classifier = self.tsc,
            scramble = False,
            fast_foward = False
        )
        for i in range(constants.config.n_tests):
            print("Test %d/%d" % (i+1, constants.config.n_tests), end="\r")
            exp.step()
        print("\nComplete!")
        return

    
 


if __name__ == "__main__":
    DriveableAreaTest()