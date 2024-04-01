import warnings
warnings.simplefilter('ignore')

import PySimpleGUI as sg
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from driveable_area import DriveableAreaEstimator

class GUI:
    def __init__(self):
        sg.theme("SystemDefault1")

        # Load Frames 
        frames = self._load_frames()

        # Configuration Frame
        layout = [
            [frames["Time"]],
            [frames["Initial State"]],
            [frames["Vehicle Properties"]],
            [frames["Control Variables"]]
        ]
        config_frame = sg.Frame("Parameters", layout)
        
        # Canvas
        self._figure_agg = None
        layout = [
            [sg.Canvas(size=(100, 100), key="-CANVAS-")]
        ]
        canvas_frame = sg.Frame("Canvas", layout)

        # Statistics
        layout = [
            [
                sg.Text("Driveable Area:"),
                sg.Multiline(
                    key="driveable-area",
                    size=(1,1),
                    expand_x=True,
                    font="Courier"
                )
            ],
            [sg.HorizontalSeparator()],
            [sg.T("Trajectories")],
            [sg.Multiline(
                key="trajectory-output", 
                size=(3,5), 
                expand_x=True,
                font = "Courier",
                horizontal_scroll=True
            )],
            [
                sg.T("Shape:"), 
                sg.Multiline(
                    key="shape-output",
                    size=(1,1),
                    expand_x = True,
                    font = "Courier",
                    horizontal_scroll = True
                )
            ]
        ]
        stats_frame = sg.Frame("Statistics", layout, expand_x=True)


        # Ribbon
        ribbon = [
            sg.Button("Generate")
            
        ]

        # Main Layout
        main_layout = [
            ribbon,
            [config_frame, canvas_frame],
            [stats_frame]
        ]

        window = sg.Window("Driveable Area", main_layout, finalize=True)
        self._window = window

        

        # Initilizae canvas
        # self.init_canvas()
        event, values = window.read(timeout=0)
        self.event_generate(values)
        
        
        # Event loop
        while True:
            event, values = window.read()
            if event == sg.WINDOW_CLOSED:
                break
            elif event == "Generate":
                try:
                    self.event_generate(values)
                except:
                    pass            

        window.close()
        return
    
    @property
    def dae(self) -> DriveableAreaEstimator:
        return self._dae

    @property
    def features(self) -> list[str]:
        return self._features
    
    @property
    def window(self) -> sg.Window:
        return self._window
    
    @property
    def figure_agg(self) -> FigureCanvasTkAgg:
        return self._figure_agg
    
    def event_generate(self, values : dict):
        kwargs = {}
        for feat in self.features:
            kwargs[feat] = values[feat]
        self._dae = DriveableAreaEstimator(**kwargs)
        self._update_figure()
        
        # Driveable Area
        self.window["driveable-area"]\
            .update(value = "%.3fm^2" % self.dae.driveable_area)
        
        self.window["trajectory-output"].update(
            value = self.dae.traj_summary_concise.to_csv(sep="\t", index=False)
        )
       
        self.window["shape-output"].update(
            value = tuple(self.dae.boundary.exterior.coords)
        )

        return
    
    def draw_figure(self, canvas, figure):
        figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
        figure_canvas_agg.draw()
        figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        return figure_canvas_agg
    
    def delete_figure_agg(self, figure_agg):
        figure_agg.get_tk_widget().forget()
        plt.close('all')
        return
    
    def blank_figure(self) -> plt.Figure:
        fig = plt.figure(figsize=(4.5,4.5))
        ax = fig.gca()
        
        x = [1, 2, 3, 4, 5]
        y = [2, 3, 5, 7, 11]

        # Create a plot
        ax.plot(x, y)
        return fig
    
    def init_canvas(self):
        fig = self.blank_figure()
        self._figure_agg = self.draw_figure(self.window['-CANVAS-'].TKCanvas, fig)
        self.window.refresh()
        self.delete_figure_agg(self.figure_agg)
        self.window.refresh()
        return

    def _update_figure(self):
        # Remove Old FIgure
        if self.figure_agg != None:
            self.delete_figure_agg(self._figure_agg)

        #  Make plot
        fig = self.dae.plot_summary()

        # Add the Matplotlib plot to the PySimpleGUI window
        self._figure_agg = self.draw_figure(self.window['-CANVAS-'].TKCanvas, fig)    
        return

    def _load_frames(self) -> dict:
        df = pd.read_csv("gui_config.csv")
        self._features = df["feat"].tolist()
        self.feat_size = int(df["feat"].apply(len).max()) + 1
        self.uom_size = int(df["uom"].dropna().apply(len).max() ) + 1
        frames = {}
        for frame_title in df["frame"].unique():
            frame_df = df[df["frame"] == frame_title]
            frames[frame_title] = self._get_frame(frame_df)
        return frames
    
    def _get_frame(self, df : pd.DataFrame) -> sg.Frame:
        layout = []
        
        for i in range(len(df.index)):
            s = df.iloc[i]
            row = [
                sg.T(
                    s["feat"], 
                    justification = "right", 
                    size = self.feat_size
                ),
                sg.Input(s["default"],size=int(s["input_size"]), key=s["feat"]),
                sg.T(
                    s["uom"],
                    justification = "left",
                    size = self.uom_size
                ),
                sg.T(s["desc"])
            ]
            layout.append(row)
            continue
        return sg.Frame(s["frame"], layout, vertical_alignment="top", expand_x=True)
    
if __name__ == "__main__":
    GUI()