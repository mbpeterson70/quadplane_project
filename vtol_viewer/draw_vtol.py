"""
vtolsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - BGM
"""
import sys
sys.path.append('..')
import numpy as np
from numpy.typing import _256Bit
import pyqtgraph.opengl as gl
from tools.rotations import Euler2Rotation


class DrawVTOL:
    def __init__(self, state, window):
        """
        Draw the VTOL.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        # get points that define the non-rotated, non-translated vtol and the mesh colors
        self.vtol_points, self.vtol_meshColors = self.get_points()

        vtol_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of vtol as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining vtol
        rotated_points = self.rotate_points(self.vtol_points, R)
        translated_points = self.translate_points(rotated_points, vtol_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        self.vtol_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.vtol_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        window.addItem(self.vtol_body)  # add body to plot

    def update(self, state):
        vtol_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of vtol as a rotation matrix R from body to inertial
        R = Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining vtol
        rotated_points = self.rotate_points(self.vtol_points, R)
        translated_points = self.translate_points(rotated_points, vtol_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(translated_points)
        # draw VTOL by resetting mesh using rotated and translated points
        self.vtol_body.setMeshData(vertexes=mesh, vertexColors=self.vtol_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def create_circle_pts(self, circle_c, circle_r, circle_n, start_unit_vector, num_triangles):
        points = np.array([[], [], []])
            
        for i in range(num_triangles):
            theta = (2*np.pi/num_triangles)*i
            omega = np.cross(circle_n.reshape(-1), start_unit_vector.reshape(-1)).reshape((3,1))
            rotated_vec = (
                np.linalg.norm(start_unit_vector) * 
                (np.cos(theta)*start_unit_vector/np.linalg.norm(start_unit_vector) +
                np.sin(theta)*omega/np.linalg.norm(omega))
            )
            point = circle_c + rotated_vec * circle_r
            points = np.concatenate([points, point], axis=1)
        return points

    def get_points(self):
        """"
            Points that define the vtol, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define vtol body parameters
        unit_length = 0.25
        fuse_h = unit_length
        fuse_w = unit_length
        fuse_l1 = unit_length * 2
        fuse_l2 = unit_length
        fuse_l3 = unit_length * 4
        wing_l = unit_length
        wing_w = unit_length * 6
        tail_h = unit_length
        tail_l = unit_length
        tail_w = unit_length * 2

        # points are in NED coordinates
        # define the points on the aircraft following diagram Fig 2.14
        # points = np.array([[0, 0, 0],  # point 1 [0]
        #                    [1, 1, 1],  # point 2 [1]
        #                    [1, 1, 0],  # point 3 [2]
        #                    ]).T
        points = np.array([[fuse_l1, 0, 0],
                           [fuse_l2, fuse_w / 2, -fuse_h / 2],
                           [fuse_l2, -fuse_w / 2, -fuse_h / 2],
                           [fuse_l2, -fuse_w / 2, fuse_h / 2],
                           [fuse_l2, fuse_w / 2, fuse_h / 2],
                           [-fuse_l3, 0, 0],
                           [0, wing_w / 2, 0],
                           [-wing_l, wing_w / 2, 0],
                           [-wing_l, -wing_w / 2, 0],
                           [0, -wing_w / 2, 0],
                           [-(fuse_l3 - tail_l), tail_w / 2, 0],
                           [-fuse_l3, tail_w / 2, 0],
                           [-fuse_l3, -tail_w / 2, 0],
                           [-(fuse_l3 - tail_l), -tail_w / 2, 0],
                           [-(fuse_l3 - tail_l), 0, 0],
                           [-fuse_l3, 0, -tail_h],
                        ]).T

        circle_n = np.array([[0, 0, 1]]).T
        circle_r = unit_length
        start_unit_vector = np.array([[1, 0, 0]]).T
        self.num_circle_triangles = 20

        circle_c = np.array([[1.25*unit_length, 1.75*unit_length, -.5*unit_length]]).T
        points = np.concatenate([points, circle_c, 
            self.create_circle_pts(circle_c, circle_r, circle_n, start_unit_vector, self.num_circle_triangles)], axis=1)
        circle_c = np.array([[1.25*unit_length, -1.75*unit_length, -.5*unit_length]]).T
        points = np.concatenate([points, circle_c, 
            self.create_circle_pts(circle_c, circle_r, circle_n, start_unit_vector, self.num_circle_triangles)], axis=1)
        circle_c = np.array([[-2.*unit_length, 1.75*unit_length, -.5*unit_length]]).T
        points = np.concatenate([points, circle_c, 
            self.create_circle_pts(circle_c, circle_r, circle_n, start_unit_vector, self.num_circle_triangles)], axis=1)
        circle_c = np.array([[-2.*unit_length, -1.75*unit_length, -.5*unit_length]]).T
        points = np.concatenate([points, circle_c, 
            self.create_circle_pts(circle_c, circle_r, circle_n, start_unit_vector, self.num_circle_triangles)], axis=1)
        circle_c = np.array([[fuse_l1, 0, 0]]).T
        circle_n = np.array([[1, 0, 0]]).T
        start_unit_vector = np.array([[0, 0, 1]]).T
        points = np.concatenate([points, circle_c, 
            self.create_circle_pts(circle_c, circle_r, circle_n, start_unit_vector, self.num_circle_triangles)], axis=1)


        # scale points for better rendering
        scale = 1
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        white = np.array([1., 1., 1., 1.])
        meshColors = np.empty((13+5*self.num_circle_triangles, 3, 4), dtype=np.float32)
        meshColors[0] = yellow # nose
        meshColors[1] = yellow # nose
        meshColors[2] = yellow # nose
        meshColors[3] = yellow # nose
        meshColors[4] = blue # body
        meshColors[5] = blue # body
        meshColors[6] = red # body
        meshColors[7] = blue # body
        meshColors[8] = blue # rudder fin
        meshColors[9] = green # wing
        meshColors[10] = green # wing
        meshColors[11] = green # tail wing
        meshColors[12] = green # tail wing
        for i in range(13, 13+5*self.num_circle_triangles):
            meshColors[i] = white

        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],
                         [points[0], points[2], points[3]],
                         [points[0], points[3], points[4]],
                         [points[0], points[1], points[4]],
                         [points[1], points[2], points[5]],
                         [points[2], points[3], points[5]],
                         [points[3], points[4], points[5]],
                         [points[4], points[1], points[5]],
                         [points[5], points[14], points[15]],
                         [points[6], points[7], points[8]],
                         [points[6], points[8], points[9]],
                         [points[10], points[11], points[12]],
                         [points[10], points[12], points[13]]
        ])
        for k in range(5):
            for i in range(self.num_circle_triangles):
                j = i+1 if i != self.num_circle_triangles - 1 else 0
                mesh = np.concatenate(
                    [mesh, np.array([[points[16+k*(self.num_circle_triangles+1)], 
                    points[17+i+k*(self.num_circle_triangles+1)], 
                    points[17+j+k*(self.num_circle_triangles+1)]]])], 
                    axis=0)

        return mesh
