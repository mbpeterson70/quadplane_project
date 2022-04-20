"""
vtolsim_python: vtol viewer (for chapter 2)
    - Beard & McLain, PUP, 2012
    - Update history:
        1/15/2019 - RWB
        4/15/2019 - BGM
        3/31/2020 - RWB
"""
import sys
sys.path.append("..")
from vtol_viewer.draw_vtol import DrawVTOL
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
# from vtol_viewer.draw_vtol import DrawVTOL


class VTOLViewer():
    def __init__(self):
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('VTOL Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=30) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_() # bring window to the front
        self.plot_initialized = False # has the vtol been plotted yet?
        self.vtol_plot = []

    def update(self, state):
        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            self.vtol_plot = DrawVTOL(state, self.window)
            self.plot_initialized = True
        # else update drawing on all other calls to update()
        else:
            self.vtol_plot.update(state)
        # update the center of the camera view to the vtol location
        view_location = Vector(state.east, state.north, state.altitude)  # defined in ENU coordinates
        self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()

    def addTrajectory(self, points):
        blue = np.array([[30, 144, 255, 255]])/255.
        self.trajectory = drawTrajectory(points, blue, self.window)

class drawTrajectory:
    def __init__(self, points, color, window):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        
        points = points.T
        self.color = color
        path_color = np.tile(color, (points.shape[0], 1))
        self.path_plot_object =  gl.GLLinePlotItem(pos=points,
                                                   color=path_color,
                                                   width=2,
                                                   antialias=True,
                                                   mode='line_strip')
        window.addItem(self.path_plot_object)

    def update(self, points):
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        points = R @ np.copy(points)
        self.path_plot_object.setData(pos=points)

