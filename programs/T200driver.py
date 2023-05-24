import time
import pigpio


class T200:
    def __init__(self, pin):
        self.pwm = pigpio.pi()
        self.pwm.set_mode(pin, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency(pin, 50)
        self.pwm.set_PWM_range(pin, 20000)  # 1,000,000 / 50 = 20,000us for 100% duty >
        self.pin = pin

    def arm_motor(self):
        self.pwm.set_servo_pulsewidth(self.pin, 1500)
        print("Motor armed")

    def set_motor_speed(self, cmd):
        self.pwm.set_servo_pulsewidth(self.pin, cmd)

    def reset(self):
        self.pwm.set_servo_pulsewidth(self.pin, 1500)
        self.pwm.stop()


if __name__ == '__main__':
    topMotor = T200(17)
    leftMotor = T200(22)
    rightMotor = T200(27)

    time.sleep(0.5)
    topMotor.arm_motor()
    time.sleep(0.5)
    leftMotor.arm_motor()
    time.sleep(0.5)
    rightMotor.arm_motor()

    while True:
        try:
            time.sleep(0.5)
            topMotor.set_motor_speed(1600)
            leftMotor.set_motor_speed(1600)
            rightMotor.set_motor_speed(1600)
        except KeyboardInterrupt:
            break

    topMotor.reset()
    leftMotor.reset()
    rightMotor.reset()
