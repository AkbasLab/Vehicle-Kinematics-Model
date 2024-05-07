import pandas as pd
import matplotlib.pyplot as plt
import os

import utils
import constants

def main():
    # scenario_graphs()
    # crunch_stats()
    # summary_md()
    entropy_comparison()
    return

def entropy_comparison():
    scenarios = [
        constants.scenario.cut_in,
        constants.scenario.no_traffic,
        constants.scenario.two_lane_traffic,
        constants.scenario.three_lane_traffic,
        constants.scenario.pedestrian_crossing
    ]
    for scenario in scenarios:
        eda = EDA(scenario)
    return

def summary_md():
    scenarios = [
        constants.scenario.cut_in,
        constants.scenario.no_traffic,
        constants.scenario.two_lane_traffic,
        constants.scenario.three_lane_traffic,
        constants.scenario.pedestrian_crossing
    ]
    
    msg = "# Results Summary"
    for scenario in scenarios:
        eda = EDA(scenario)
        msg += eda.img_markdown
        continue

    with open("result-summary.md","w") as f:
        f.write(msg)
    return

def crunch_stats():
    scenarios = [
        constants.scenario.cut_in,
        constants.scenario.no_traffic,
        constants.scenario.two_lane_traffic,
        constants.scenario.three_lane_traffic,
        constants.scenario.pedestrian_crossing
    ]
    
    data = []

    for scenario in scenarios:
        work_dir = "results/%s" % scenario
        df = pd.read_csv("%s/score_hist.csv" % work_dir)

        total = 10_000
        
        if scenario == constants.scenario.no_traffic:
            n_collision = 0
            n_brake = len(df[
                (df["max(decel.normal.A)"] <= 1.0)
            ])
            n_ebrake = len(df[
                (df["max(decel.normal.A)"] > 1.0)
            ])
        else:
            n_collision = len(df[df["collision.A"] > 0].index)
            n_brake = len(df[
                (df["collision.A"] == 0) &
                (df["max(decel.normal.A)"] <= 1.0)
            ])
            n_ebrake = len(df[
                (df["collision.A"] == 0) &
                (df["max(decel.normal.A)"] > 1.0)
            ])

        s = pd.Series({
            "scenario" : scenario,
            "# tests" : 10_000,
            "collision" : n_collision,
            "normal" : n_brake,
            "near collision" : n_ebrake,
        })
        data.append(s)
        continue

    df = pd.DataFrame(data)
    
    for feat in ["collision", "normal", "near collision"]:
        df["%s %%" % feat] = df[feat]/10000 * 100

    print("\n\nCollision")
    print(df.sort_values(by="collision"))

    print("\n\nNormal")
    print(df.sort_values("normal"))

    print("\n\nNear Collision")
    print(df.sort_values("near collision"))

    df.sort_values(by="collision", ascending=False)\
        .to_csv("out/ranking.csv", index=False)
    return

def scenario_graphs():
    scenarios = [
        constants.scenario.cut_in,
        constants.scenario.no_traffic,
        constants.scenario.two_lane_traffic,
        constants.scenario.three_lane_traffic,
        constants.scenario.pedestrian_crossing
    ]

    for scenario in scenarios:
        eda = EDA(scenario)
        eda.generate_plots()
    return

class EDA:
    def __init__(self, scenario : str):
        """
        Load data
        """
        # self.scenario = "cut-in"
        self.scenario = scenario

        # Load
        self.work_dir = "results/%s" % self.scenario
        param_df = pd.read_csv("%s/param_hist.csv" % self.work_dir)
        score_df = pd.read_csv("%s/score_hist.csv" % self.work_dir)

        # Combine Dataframes
        self.df = pd.concat([param_df,score_df], axis = 1, sort=False)



        """
        Generate Lattice Plots
        """
        self.dpi = 300
        self.img_dir = "%s/img" % self.work_dir
        # print(self.df)
        # self.generate_plots()

        """
        Markdown Summary
        """
        self.img_markdown = self._markdown_summary()
        return
    
    def _markdown_summary(self) -> str:
        # Get all filenames of imgs
        fns = [f for f in os.listdir(self.img_dir) \
               if os.path.isfile(os.path.join(self.img_dir, f))]
        
        msg = "\n\n## %s\n" % self.scenario
        for fn in fns:
            fn = fn.replace("(", "\(")
            fn = fn.replace(")", "\)")

            msg += "\n### %s\n" % fn[:-4]
    
            msg += "![](%s/%s)\n" % (self.img_dir, fn)
        
        msg = msg[:-1]
        return msg
    
    def generate_plots(self):
        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)
        # df = self.df
        # print(self.df)

        # # Collission Stats
        unsafe_df = self.df[self.df["collision.A"] > 0]
        prefix = "unsafe"
        self.fast_grid_scatter_plot(
            unsafe_df,
            prefix,
            exclude = ["entropy.A","collision.A","max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 2
        )
        self._fast_lattice_plots(
            unsafe_df,
            prefix = prefix,
            exclude = "collision.A",
            trim = 0.1
        )



        # # No Collission Scenarios
        safe_df = self.df[self.df["collision.A"] == 0]
        prefix = "safe"
        self.fast_grid_scatter_plot(
            safe_df,
            prefix,
            exclude = ["entropy.A","collision.A","max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 2
        )
        self._fast_lattice_plots(
            safe_df,
            prefix = prefix,
            exclude = "collision.A",
            trim = 0.1
        )


        
        # # Comfortable Break Force
        brake_df = self.df[self.df["max(decel.normal.A)"] <= 1.0]
        prefix = "brake"
        self.fast_grid_scatter_plot(
            brake_df,
            prefix,
            exclude = ["entropy.A", "max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 3
        )
        self._fast_lattice_plots(
           brake_df,
           prefix = prefix,
           exclude =  "max(decel.normal.A)"
        )



        # # Emergency Brake / uncomfortable
        ebrake_df = self.df[self.df["max(decel.normal.A)"] > 1.0]
        prefix = "ebrake"
        self.fast_grid_scatter_plot(
            ebrake_df,
            prefix = prefix,
            exclude = ["entropy.A", "max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 3
        )
        self._fast_lattice_plots(
           ebrake_df,
           prefix = prefix,
           exclude =  "max(decel.normal.A)"
        )





        # # No collision, comfortable braking
        safe_brake_df = self.df[
                (self.df["collision.A"] == 0 ) \
                & (self.df["max(decel.normal.A)"] <= 1.0)
        ]
        prefix = "safe_brake"
        self.fast_grid_scatter_plot(
            safe_brake_df,
            prefix = prefix,
            exclude = ["collision.A", "entropy.A", "max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 2
        )

        self._fast_lattice_plots(
           safe_brake_df,
            prefix = "safe_brake",
            exclude = ["collision.A", "max(decel.normal.A)"]
        )


        # No collision, emergency braking
        safe_ebrake_df = self.df[
            (self.df["collision.A"] > 0 ) \
            & (self.df["max(decel.normal.A)"] > 1.0)
        ]
        prefix = "safe_ebrake"
        self.fast_grid_scatter_plot(
            safe_ebrake_df,
            prefix = prefix,
            exclude = ["collision.A", "entropy.A", "max(decel.normal.A)"],
            fig_size = [10,7],
            nrow = 2
        )
        
        self._fast_lattice_plots(
            safe_ebrake_df,
            prefix = "safe_ebrake",
            exclude = ["collision.A", "max(decel.normal.A)"]
        )

        return
    
    def fast_grid_scatter_plot(self,
            df : pd.DataFrame, 
            prefix : str, 
            fig_size : list[float,float],
            exclude : str = "",
            nrow : int = None,
            ncol : int = None
        ):
        if not isinstance(exclude, list):
            exclude = [exclude]
        features = [feat for feat in constants.scores[self.scenario] \
                    if not feat in exclude]
        img_fn = "%s/%s-entropy_scatter.png" % (self.img_dir, prefix)
        utils.scatter_grid_plot(
            df,
            features = features,
            score_feat = "entropy.A",
            fig_size = fig_size,
            nrow = nrow,
            ncol = ncol,
            img_fn = img_fn,
            dpi = self.dpi
        )
        return
    
    def _fast_lattice_plots(self, 
            df : pd.DataFrame, 
            prefix : str, 
            exclude : str = "",
            trim : float = 0.
        ):
        if not isinstance(exclude, list):
            exclude = [exclude]
        scores_to_test = [score for score in constants.scores[self.scenario] \
                        if not score in exclude]
        for score in scores_to_test:
            img_fn = "%s/%s-%s.png" % (self.img_dir, prefix, score)
            print("Generating plot %s" % img_fn)
            utils.lattice_plot(
                df,
                features = constants.features[self.scenario],
                score_feat = score,
                fig_size= [5,5],
                dpi = self.dpi,
                img_fn = img_fn,
                trim = trim
            )
        
        plt.close()
        return



if __name__ == "__main__":
    main()