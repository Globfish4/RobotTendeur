import ms5837
import time


class PressureSensor:
    def __init__(self):
        self.sensor = ms5837.MS5837() # Use defaults (MS5837-30BA device on I2C bus 1)
        self.isConnected = False
        """
        try:
            self.isConnected = True
            self.sensor.init()
        except OSError:
            print("Unable to connect to Pressure sensor")
            self.isConnected = False

        if self.isConnected:
            self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER) # Use predefined saltwater/fr>
        """


    def synchronize(self):
        if not self.isConnected:
            try:
                self.sensor.init()
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
            except OSError:
                print("Unable to connect to Pressure sensor")
            else:
                self.isConnected = True
                print("Pressure sensor connected!")
        elif self.isConnected:
            print("Already connected to Pressure sensor")
        """
        elif self.isConnected:
            try:
                self.sensor.read()
            except OSError:
                print("Unable to read from Pressure sensor")
            except AttributeError:
                print("Trying to reconnect to Pressure sensor")
                self.sensor.init()
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        """



    def getMeasure(self):
        pressure, temperature = 0, 0
        if self.isConnected:
            try:
                self.sensor.read()
            except OSError:
                print("Unable to read from Pressure sensor")
                pass
            except AttributeError:
                pass
                #print("Trying to reconnect to Pressure sensor")
            pressure = self.sensor.pressure()
            temperature = self.sensor.temperature()
        else:
            print("please, reconnect to Pressure sensor")
        return pressure, temperature



if __name__ == '__main__':
    my_sensor = PressureSensor()
    my_sensor.synchronize()
    running = True

    while running:
        pressure, temperature = my_sensor.getMeasure()
        print(pressure, temperature)
        time.sleep(0.5)




