#/usr/bin/python3
import sys
import numpy as np

sys.path.append('..')
from vtol_trajectory_generator.trajectory_generator import LineSegment, TrajectoryGenerator

# straight_slow = LineSegment(
#     start_pos=np.array([[0, 0, 0]]).T, 
#     start_vel=0, 
#     end_pos=np.array([[100, 0, 0]]).T, 
#     end_vel=5)

# straight_fast = LineSegment(
#     start_pos=np.array([[0, 0, 0]]).T, 
#     start_vel=0, 
#     end_pos=np.array([[200, 0, 0]]).T, 
#     end_vel=15)

# take_off = LineSegment(
#     start_pos=np.array([[0, 0, 0]]).T, 
#     start_vel=0, 
#     end_pos=np.array([[200, 0, -50]]).T, 
#     end_vel=15)

tc_slow = TrajectoryGenerator()
tc_slow.add_line_segment(LineSegment(
    start_pos=np.array([[0, 0, 0]]).T, 
    start_vel=0, 
    end_pos=np.array([[100, 0, -20]]).T, 
    end_vel=5))
tc_slow.add_line_segment(LineSegment(
    start_pos=np.array([[100, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[500, 0, -20]]).T, 
    end_vel=5))

# tcl = TrajectoryGenerator()
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[0, 0, 0]]).T, 
#     start_vel=0, 
#     end_pos=np.array([[100, 0, -20]]).T, 
#     end_vel=5))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[100, 0, -20]]).T, 
#     start_vel=5, 
#     end_pos=np.array([[500, 0, -20]]).T, 
#     end_vel=10))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[500, 0, -20]]).T, 
#     start_vel=10, 
#     end_pos=np.array([[800, 0, -20]]).T, 
#     end_vel=10))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[800, 0, -20]]).T, 
#     start_vel=10, 
#     end_pos=np.array([[1200, 0, -20]]).T, 
#     end_vel=5))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[1200, 0, -20]]).T, 
#     start_vel=5, 
#     end_pos=np.array([[1600, 0, 0]]).T, 
#     end_vel=0))

# tcl = TrajectoryGenerator()
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[0, 0, 0]]).T, 
#     start_vel=0, 
#     end_pos=np.array([[100, 0, -20]]).T, 
#     end_vel=5))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[100, 0, -20]]).T, 
#     start_vel=5, 
#     end_pos=np.array([[300, 0, -20]]).T, 
#     end_vel=10))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[300, 0, -20]]).T, 
#     start_vel=10, 
#     end_pos=np.array([[500, 0, -20]]).T, 
#     end_vel=10))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[500, 0, -20]]).T, 
#     start_vel=10, 
#     end_pos=np.array([[700, 0, -20]]).T, 
#     end_vel=5))
# tcl.add_line_segment(LineSegment(
#     start_pos=np.array([[700, 0, -20]]).T, 
#     start_vel=5, 
#     end_pos=np.array([[800, 0, 0]]).T, 
#     end_vel=0))

tcl = TrajectoryGenerator()
tcl.add_line_segment(LineSegment(
    start_pos=np.array([[0, 0, 0]]).T, 
    start_vel=0, 
    end_pos=np.array([[50, 0, -20]]).T, 
    end_vel=5))
tcl.add_line_segment(LineSegment(
    start_pos=np.array([[50, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[150, 0, -20]]).T, 
    end_vel=10))
tcl.add_line_segment(LineSegment(
    start_pos=np.array([[150, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[250, 0, -20]]).T, 
    end_vel=10))
tcl.add_line_segment(LineSegment(
    start_pos=np.array([[250, 0, -20]]).T, 
    start_vel=10, 
    end_pos=np.array([[350, 0, -20]]).T, 
    end_vel=5))
tcl.add_line_segment(LineSegment(
    start_pos=np.array([[350, 0, -20]]).T, 
    start_vel=5, 
    end_pos=np.array([[400, 0, 0]]).T, 
    end_vel=0))



# ls = LineSegment(
#     start_pos=np.array([[100, 0, -20]]).T, 
#     start_vel=5, 
#     end_pos=np.array([[500, 0, -20]]).T, 
#     end_vel=5)
    
# for t in range(100):
#     t = t/100*tc_slow.end_time
#     print(tc_slow.position(t).T)

# for t in range(100):
#     t = t/100*ls.time
#     print(ls.position(t).T)