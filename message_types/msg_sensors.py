"""
msg_sensors
    - messages type for output of sensors
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/16/2019 - RWB
"""


class MsgSensors:
    def __init__(self):
        self.gyro_x = 0  # gyroscope along body x axis
        self.gyro_y = 0  # gyroscope along body y axis
        self.gyro_z = 0  # gyroscope along body z axis
        self.accel_x = 0  # specific acceleration along body x axis
        self.accel_y = 0  # specific acceleration along body y axis
        self.accel_z = 0  # specific acceleration along body z axis
        self.mag_x = 0  # magnetic field along body x axis
        self.mag_y = 0  # magnetic field along body y axis
        self.mag_z = 0  # magnetic field along body z axis
        self.abs_pressure = 0  # absolute pressure
        self.diff_pressure = 0  # differential pressure
        self.gps_n = 0  # gps north
        self.gps_e = 0  # gps east
        self.gps_h = 0  # gps altitude
        self.gps_Vg = 0  # gps ground speed
        self.gps_course = 0  # gps course angle
    
    def __str__(self):
        return f'fgyro_x: {self.gyro_x}\n' + \
            f'gyro_y: {self.gyro_y}\n' + \
            f'gyro_z: {self.gyro_z}\n' + \
            f'accel_x: {self.accel_x}\n' + \
            f'accel_y: {self.accel_y}\n' + \
            f'accel_z: {self.accel_z}\n' + \
            f'mag_x: {self.mag_x}\n' + \
            f'mag_y: {self.mag_y}\n' + \
            f'mag_z: {self.mag_z}\n' + \
            f'abs_pressure: {self.abs_pressure}\n' + \
            f'diff_pressure: {self.diff_pressure}\n' + \
            f'gps_n: {self.gps_n}\n' + \
            f'gps_e: {self.gps_e}\n' + \
            f'gps_h: {self.gps_h}\n' + \
            f'gps_Vg: {self.gps_Vg}\n' + \
            f'gps_course: {self.gps_course}\n'