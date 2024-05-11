import matplotlib.pyplot as plt
class DataCollector:
    def __init__(self):
        self.data = {
            'time': [],
            'speed': [],
            'throttle': [],
            'distance': [],
            'acceleration': []
        }
    
    def append(self, time, speed, throttle, distance, acceleration):
        self.data['time'].append(time)
        self.data['speed'].append(speed)
        self.data['throttle'].append(throttle)
        self.data['distance'].append(distance)
        self.data['acceleration'].append(acceleration)

    def get_data(self):
        return self.data

    def PlotData(self):
        plt.plot(self.data['time'], self.data['speed'])
        plt.xlabel('Time (s)')
        plt.ylabel('Ego Vehicle Speed (m/s)')
        plt.title('Ego Vehicle Speed as a Function of Time')
        plt.grid(True)

        plt.figure()
        plt.plot(self.data['time'],self.data['distance'])
        plt.xlabel('Time (s)')
        plt.ylabel('Distance between ego and target')
        plt.title('Distance between ego and target as a Function of Time')
        plt.grid(True)

        plt.figure()
        plt.plot(self.data['time'], self.data['acceleration'])
        plt.xlabel('Time (s)')
        plt.ylabel('Acc of ego')
        plt.title('Acc of ego  as a Function of Time')
        plt.grid(True)


        plt.figure()
        plt.plot(self.data['time'],  self.data['throttle'])
        plt.xlabel('Time (s)')
        plt.ylabel('throttle ego')
        plt.title('throttle as a Function of Time')
        plt.grid(True)


        plt.show()