import sys
import time
import socket
import threading

import PSdriver
import IMUdriver
import T200driver

import numpy as np
from scipy.linalg import expm, logm
from numpy.linalg import inv

# communication by socket with the simulation
robot_IP_ADR = "172.20.13.109"
IP_ADR = "172.20.20.88"
PORT = 12345

# Mutex initialisation
IMULock = threading.Lock()
PSLock = threading.Lock()
cmdLock = threading.Lock()


class Rob:
    def __init__(self):
        self.imu = IMUdriver.IMU()
        self.pressureSensor = PSdriver.PressureSensor()  # may not succeed to connect...
        self.topMotor = T200driver.T200(17)
        self.leftMotor = T200driver.T200(27)
        self.rightMotor = T200driver.T200(22)

        # Current motor angular speed
        # 1000 : max backward speed
        # 1500 : stop
        # 2000 : max forward speed
        self.topMotorSpd = 1500
        self.leftMotorSpd = 1500
        self.rightMotorSpd = 1500

        # Define some constants used for conversion
        self.permanent = 50
        self.delta = 100
        self.u0 = 1e4
        self.dp0 = np.pi

        self.updateTime = 0.1  # update sensor & command at 10Hz

        self.askIMU = False
        self.askPS = False

        self.running = True
        self.manualCmd = False  # driven by the simulation by default

        self.IMUThread = None
        self.PSThread = None
        self.T200Thread = None
        self.controlThread = None
        self.inputThread = None

        # Define parameters for auto control
        self.root = 3  # dynamic equation root

        self.R = angles2dcm(0, 0, 0.1)  # rotation matrix
        self.wr = np.array([[0], [0], [0]])  # robot angular speed in its own frame

        self.R_d = angles2dcm(0, 0, 0)  # desired rotation matrix : cap command (in radians)
        self.m = 4.8  # robot mass in kilograms
        self.g = 9.81  # gravitational constant in m.s-2
        self.m_lest = 0.7  # lest mass in kilograms

        self.rot_fr = 5e-2  # frictional resistance of water against rotations

        self.L = 0.4  # length of the robot in meters
        self.l = 0.15  # distance of propellers from robot axis
        self.radius = 0.06  # robot radius in meters

        # inertial matrix of the robot
        self.I = np.diag([(1 / 2) * self.m * self.radius ** 2,
                          (1 / 4) * self.m * self.radius ** 2 + (1 / 12) * self.m * self.L ** 2,
                          (1 / 4) * self.m * self.radius ** 2 + (1 / 12) * self.m * self.L ** 2])

        # Define links between propellers and mechanical actions
        kCx = 1e-3  # rotation around x axis can't be control
        kCy = 1e-1
        kCz = 1e-1

        # to convert commands to propellers speed
        self.B = np.array([[-kCx * self.l, kCx * self.l, kCx * self.l],  # first motor turn in opposite direction
                           [kCy * self.l, (-np.sqrt(3) / 2) * kCy * self.l, (-np.sqrt(3) / 2) * kCy * self.l],
                           [0, (+1 / 2) * kCz * self.l, (-1 / 2) * kCz * self.l]])

        self.u = np.array([[0], [0], [0]])  # motor commands

    def launch_threads(self, md=None):
        self.IMUThread = threading.Thread(target=self.imu_t)
        self.IMUThread.start()

        self.PSThread = threading.Thread(target=self.pressure_sensor_t)
        self.PSThread.start()

        self.T200Thread = threading.Thread(target=self.propellers_t)
        self.T200Thread.start()

        self.controlThread = threading.Thread(target=self.control_robot_basic_t)
        self.controlThread.start()

        if md is None:
            self.inputThread = threading.Thread(target=self.user_interface_t)
            self.inputThread.start()

    def reset(self, md=None):
        self.IMUThread.join()
        self.PSThread.join()
        self.T200Thread.join()
        self.controlThread.join()
        if md is None:
            self.inputThread.join()

        self.imu.reset()
        self.topMotor.reset()
        self.leftMotor.reset()
        self.rightMotor.reset()

        print("Robot shutdown...")

    def imu_t(self):
        while self.running:
            self.imu.update()  # block until data is received
            if self.askIMU:
                self.askIMU = False
                print('gyro : ', self.imu.data['gyroscope'])
                print('rot : ', self.imu.data['rotation'])

    def pressure_sensor_t(self):
        while self.running:
            if self.pressureSensor.isConnected:
                if self.askPS:
                    self.askPS = False
                    pressure, temperature = self.pressureSensor.getMeasure()
                    print("Pressure :", pressure, "Temperature :", temperature)
            else:
                if self.askPS:
                    self.askPS = False
                    print("Unable to connect to Pressure sensor")
                    print("trying to synchronize...")
                    self.pressureSensor.synchronize()
            time.sleep(self.updateTime)

    def propellers_t(self):
        self.topMotor.arm_motor()
        self.leftMotor.arm_motor()
        self.rightMotor.arm_motor()

        while self.running:
            cmdLock.acquire(blocking=True)  # this tread has nothing else to do -> timeout=0.0

            self.topMotor.set_motor_speed(self.topMotorSpd)
            self.leftMotor.set_motor_speed(self.leftMotorSpd)
            self.rightMotor.set_motor_speed(self.rightMotorSpd)

            cmdLock.release()
            time.sleep(self.updateTime)

    def motor_control2propeller_speed(self, u):
        # print('u', u)
        if u > 0:
            return 1500 + self.permanent + self.delta * (1 - np.exp(-u/self.u0))
        elif u < 0:
            return 1500 - self.permanent - self.delta * (1 - np.exp(+u/self.u0))
        else:
            return 1500 + self.permanent

    def control_robot_t(self):
        while self.running:
            if not self.manualCmd:
                self.update_robot()  # update pose & speed (R & wr)
                self.orientation_control()  # update motor control (u)
                u_top, u_left, u_right = self.u.flatten()
                w_top, w_left, w_right = (self.motor_control2propeller_speed(u)
                                          for u in [u_top, u_left, u_right])

                print("CMD:", w_top, w_left, w_right)

                cmdLock.acquire(blocking=True)
                self.topMotorSpd = w_top
                self.leftMotorSpd = w_left
                self.rightMotorSpd = w_right
                cmdLock.release()

                time.sleep(self.updateTime)

    def control_robot_basic_t(self):
        while self.running:
            if not self.manualCmd:
                self.update_robot()  # update pose & speed (R & wr)
                phi, theta, psi = dcm2angles(self.R)
                phi_bar, theta_bar, psi_bar = dcm2angles(self.R_d)

                dw = self.delta * (1 - np.exp(-((psi_bar - psi) / self.dp0)**2))
                dw += self.step(dw)

                w_top = 1500
                w_left = 1500 - dw
                w_right = 1500 + dw

                print("CMD:", w_top, w_left, w_right)

                cmdLock.acquire(blocking=True)
                self.topMotorSpd = w_top
                self.leftMotorSpd = w_left
                self.rightMotorSpd = w_right
                cmdLock.release()

                time.sleep(self.updateTime)

    def user_interface_t(self):
        time.sleep(3.5)  # waiting for initialization
        while self.running:
            time.sleep(self.updateTime)
            if self.manualCmd:
                line = input("Manual>>>")
            else:
                line = input("Auto>>>")

            if line[:3] == 'get':
                self.update_robot()
                self.askIMU = True
                self.askPS = True
            elif line[:3] == 'cmd':
                print(self.u)
            elif line[:4] == 'stop':
                self.running = False
            elif line[:6] == 'manual':
                self.manualCmd = True
                cmdLock.acquire(blocking=True)
                self.topMotorSpd = 1500
                self.leftMotorSpd = 1500
                self.rightMotorSpd = 1500
                cmdLock.release()
            elif line[:4] == 'auto':
                self.manualCmd = False
            elif line[:3] == 'cap':
                self.R_d = angles2dcm(0, 0, eval(line[4:]) * (180/np.pi))
            elif line[:4] == 'help':
                help_tool()
            elif self.manualCmd:
                cmdLock.acquire(blocking=True)
                if line[:1] == 'T':
                    self.topMotorSpd = eval(line[1:])
                elif line[:1] == 'L':
                    self.leftMotorSpd = eval(line[1:])
                elif line[:1] == 'R':
                    self.rightMotorSpd = eval(line[1:])
                else:
                    print("Invalid command!")
                cmdLock.release()
            else:
                print("invalid command!")

    # robot_tenders.py functions
    def lest_torque(self):
        phi, theta, psi = dcm2angles(self.R)
        return (-self.radius * self.m_lest * self.g) * np.array([[np.sin(phi)], [np.sin(theta)], [0]])

    def update_robot(self):
        # angles = self.imu.get_data("euler_angles")
        # self.R = angles2dcm(angles[0], angles[1], angles[2])
        self.R = self.imu.get_data("rotation") @ angles2dcm(np.pi, 0, 0)
        self.wr = self.imu.get_data("gyroscope")

        phi, theta, psi = dcm2angles(self.R)
        print(np.array([phi, theta, psi]) * (180 / np.pi))

    def orientation_control(self):
        # print('wr', self.wr)
        w_rdp = self.root ** 2 * self.R.T @ logw(self.R_d @ self.R.T) - 2 * self.root * self.wr
        Cr = self.I @ w_rdp + adjunct(self.wr) @ self.I @ self.wr + self.lest_torque() - self.rot_fr * self.wr
        self.u = inv(self.B) @ Cr
        # print('u', self.u)

    def step(self, x):
        if x > 0:
            return +self.permanent
        elif x < 0:
            return -self.permanent
        else:
            return x


# rob-lib functions
def adjunct(w):
    if isinstance(w, (float, int)):
        return np.array([[0, -w], [w, 0]])
    elif isinstance(w, list):
        return np.array([[0, -w[2], w[1]],
                         [w[2], 0, -w[0]],
                         [-w[1], w[0], 0]],
                        dtype=object)
    elif isinstance(w, np.ndarray):
        wx, wy, wz = w.flatten()
        return adjunct([wx, wy, wz])
    else:
        print("What is that?")


def adjunct_inv(A):
    if np.size(A) == 4:
        return A[1, 0]  # A is 2x2
    else:
        return np.array([[A[2, 1]], [A[0, 2]], [A[1, 0]]])  # A is 3x3


def expw(w): return expm(adjunct(w)).astype(dtype=np.float)
def logw(R): return adjunct_inv(logm(R)).astype(dtype=np.float)


def dcm2angles(R):
    phi = np.arctan2(R[2, 1], R[2, 2])
    theta = -np.arcsin(R[2, 0])
    psi = np.arctan2(R[1, 0], R[0, 0])
    return phi, theta, psi


def angles2dcm(phi, theta, psi):
    return expw([0, 0, psi]) @ expw([0, theta, 0]) @ expw([phi, 0, 0])


def help_tool():
    print("################################################################")
    print("mode : Manual")
    print("Control the robot's motors with command inputs.")
    print("mode : Auto")
    print("Control the robot by heading command. The robot regulates itself.")
    print("\n")
    print("T____        Controls the Top motor")
    print("L____        Controls the left motor")
    print("R____        Controls the right motor")
    print("\n")
    print("    ____ is a number defined between 1000 and 2000")
    print("    1500 corresponds to the neutral position of the motor")
    print("\n")
    print("get          Get the data from IMU and pressure sensor")
    print("cmd          Get the current motor commands")
    print("cap___       Set cap in degrees the robot should follow")
    print("\n")
    print("manual       Switch mode to manual control")
    print("auto         Switch mode to auto control")
    print("stop         Shutdown the robot")
    print("################################################################")
    print("\n")


if __name__ == '__main__':
    try:
        mission_delay = int(sys.argv[1])
    except IndexError:
        mission_delay = 0

    try:
        mission_cap = float(sys.argv[2]) * (np.pi / 180)
    except IndexError:
        mission_cap = 0

    try:
        mission_duration = int(sys.argv[3])
    except IndexError:
        mission_duration = None

    time.sleep(mission_delay)
    robot = Rob()

    # try:
    #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     s.connect((IP_ADR, PORT))
    # except ConnectionRefusedError:
    #     robot.manualCmd = True
    # else:
    #     print("connected!")

    robot.launch_threads(mission_duration)

    t_init = time.time()
    while robot.running:
        if mission_duration and time.time() - t_init > mission_duration:
            robot.running = False

    robot.reset(mission_duration)
