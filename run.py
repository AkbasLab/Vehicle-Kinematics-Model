import warnings
warnings.simplefilter('ignore')

import scenarioxp as sxp
import sumo
import constants

import traci
import pandas as pd
import numpy as np
import time
import os
from typing import Callable


class DriveableAreaTest:
    def __init__(self):
        """
        Timing
        """
        start_time = time.time()

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


        # Container for parameter and score history
        self._ls_param_history = []
        self._ls_score_history = []


        preprocessing_completed_time = time.time()
        
        """
        Choose Strategy
        """
        if constants.config.strategy == constants.strategy.monte_carlo:
            self.monte_carlo_strategy()
        else:
            raise NotImplementedError("Strategy %s not supported." % constants.config.strategy)

        traci.close()

        scenario_simulation_completed_time = time.time()

        """
        Postprocessing
        """
        param_history = pd.concat(self._ls_param_history)
        score_history = pd.concat(self._ls_score_history)
        
        out_dir = "out/tests/%s" % constants.config.scenario
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        
        param_history.to_csv("%s/param_hist.csv" % out_dir, index=False)
        score_history.to_csv("%s/score_hist.csv" % out_dir, index=False)

        """
        Statistics
        """
        end_time = time.time()
        preprocessing_time = preprocessing_completed_time - start_time
        scenario_simulation_time = scenario_simulation_completed_time \
            - preprocessing_completed_time
        total_time = end_time - start_time
        
        print("      Runtime: %.3fs" % total_time)
        print("Preprocessing: %.3fs" % preprocessing_time)
        print("Sim Execution: %.3fs" % scenario_simulation_time)
        print(" Avg Sim Time: %.3fs" % 
              (scenario_simulation_time / constants.config.n_tests))
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
            print("Test %d/%d %.2f%%" % (
                i+1, 
                constants.config.n_tests, 
                (i+1)/constants.config.n_tests * 100
            ), end="\r")
            exp.step()

        self._ls_param_history.append(exp.params_history)
        self._ls_score_history.append(exp.score_history)
        print("\nComplete!")
        return

    
 


if __name__ == "__main__":
    DriveableAreaTest()