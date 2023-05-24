import time
import numpy as np
from scanf import scanf
from serial import Serial
from ahrs import Quaternion

"""
Robot referential (R1)
XYZ : East-South-Down
"""


class IMU:
    def __init__(self, port='ttyUSB0', baud_rate=115200):
        self.serial = Serial('/dev/' + port, baud_rate)  # open serial port
        print('Serial connection initialized on port', self.serial.name)

        # rtcDate rtcTime Q9_1 Q9_2 Q9_3 HeadAcc RawGX RawGY RawGZ output_Hz
        self.data = {'date': '01/01/1970',
                     'time': '00:00:00:00',
                     'quaternion': Quaternion(),
                     'rotation': np.eye(3),
                     'euler_angles': [0., 0., 0.],
                     'gyroscope': np.zeros(shape=(3, 1)),
                     'logging_rate': 10}

    def update(self):
        while True:
            try:
                line = self.serial.readline().decode('utf-8')
                rtc_date, rtc_time, qx, qy, qz, _, gx, gy, gz, output_hz = scanf\
                    ('%s,' + '%s,' + '%f,%f,%f,' + '%f,' + '%f,%f,%f,' + '%f,', line)
            except TypeError:
                print('Waiting the IMU...')
            else:
                break

        qtn = Quaternion()
        try:
            q0 = np.sqrt(1 - (qx**2 + qy**2 + qz**2))
            qtn = Quaternion(np.array([q0, qx, qy, qz]))
        except RuntimeWarning:
            print('invalid data')

        self.data['date'] = rtc_date
        self.data['time'] = rtc_time
        self.data['quaternion'] = qtn
        self.data['rotation'] = qtn.to_DCM()
        self.data['euler_angles'] = qtn.to_angles()
        self.data['gyroscope'] = np.array([[gx], [gy], [gz]]) * (np.pi/180)
        self.data['logging_rate'] = output_hz

    def get_data(self, key):
        if key in self.data.keys():
            return self.data[key]
        else:
            print('Unknown key.')

    def reset(self):
        self.serial.close()


if __name__ == '__main__':
    # from roblib import figure, ToH, circle3H, tran3H, rot3H, eulerH, draw3H, draw_arrow3D, cylinder3H, \
    #     clean3D, pause, norm
    #
    # side = 1  # pool side in meters
    #
    # L = 0.3  # length of the robot in meters
    # l = 0.2  # distance of propellers from robot axis
    # radius = 0.05  # robot radius in meters
    # blds = 0.1  # blades size
    #
    # alpha = np.array([[0], [0], [0]])  # angles for the blades
    #
    # fig = figure()
    # ax = fig.add_subplot(111, projection='3d')
    #
    # def draw_robot():
    #     a0, a1, a2 = alpha.flatten()
    #     T0 = ToH(R)  # rotate
    #     P0 = np.hstack((circle3H(blds), [[+blds, -blds], [0, 0], [0, 0], [1, 1]]))  # disc + blades
    #
    #     C0 = tran3H(L / 2, 0, l) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a0) @ P0  # we rotate the blades
    #     C1 = tran3H(L / 2, l * np.sin((2 / 3) * np.pi), l * np.cos((2 / 3) * np.pi)) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a1) @ P0
    #     C2 = tran3H(L / 2, l * np.sin((4 / 3) * np.pi), l * np.cos((4 / 3) * np.pi)) @ rot3H(0, 1.57, 0) @ eulerH(0, 0, a2) @ P0
    #
    #     draw3H(ax, T0 @ C0, 'green')
    #     draw3H(ax, T0 @ C1, 'red')
    #     draw3H(ax, T0 @ C2, 'red')
    #
    #     M = T0 @ tran3H(-L / 2, 0, 0) @ cylinder3H(radius, L)
    #     ax.plot(M[0], M[1], 1 * M[2], color='orange')
    #
    #     nwr = norm(wr)  # value between 50 (noise) & 5000 (high speed)
    #     if nwr > 100:
    #         wx, wy, wz = (wr * (side / nwr)).flatten()
    #         draw_arrow3D(ax, 0, 0, 0, wx, wy, wz, col='black')
    #
    #     pause(1e-4)

    # my_imu = IMU('cu.usbserial-110')
    my_imu = IMU()
    running = True

    while running:
        try:
            my_imu.update()

            R = my_imu.get_data('rotation')
            euler_angles = my_imu.get_data('euler_angles')
            wr = R @ my_imu.get_data('gyroscope')

            # clean3D(ax, -side/2, +side/2, -side/2, +side/2, -side/2, +side/2)
            # draw_robot()
            #
            # alpha = alpha + 1e-1
            print((np.array(euler_angles) * (180 / np.pi)).astype(np.int32))
        except KeyboardInterrupt:
            running = False

    my_imu.reset()  # close port
