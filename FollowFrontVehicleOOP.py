import glob
import math
import os
import sys
import carla
import random
import time
import cv2
from DataHandler import DataCollector
import numpy as np

try:
    sys.path.append('C:/Users/acer/Downloads/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla')
except IndexError:
    pass

class CarlaController:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.object_list =[]
        self.sensor_list = []

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.org = (30, 30)
        self.org2 = (30, 50)
        self.org3 = (30, 70)
        self.org4 = (30, 90)
        self.fontScale = 0.5
        self.color = (255, 255, 255)
        self.thickness = 1
        self.camera_data = {}
        self.vehicle={}
        self.prev_error = 0
        self.sum_error = 0
        self.time_list = []
        self.speed_list = []
        self.throttle_list = []
        self.distance_list = []
        self.acc_list = []
        self.WANTED_DISTANCE = 40
        self.dt = 1.0/20.0

        self.loc=0
        self.vel=0
        self.acc=0

        self.target_speed = 50.0 / 3.6

        self.data_collector = DataCollector()

    def set_throttle(self,distance):
    
        MAX_ERR = 50

        KP = 1
        KD = 0
        KI = 0

        err = distance - self.WANTED_DISTANCE

        derivative = (err - self.prev_error) / self.dt
        self.sum_error += err * self.dt

        throttle = KP * err + KD * derivative + KI * self.sum_error
        self.prev_error = err
        
        estimated_throttle = max(0, min(1, throttle / MAX_ERR))
        self.vehicle.apply_control(carla.VehicleControl(throttle=estimated_throttle,
                                    steer=0))
        return estimated_throttle


    def camera_callback(self,image,data_dict):
        data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

    def CleanActors(self):
        for actor in self.object_list:
            actor.destroy()
        for sensor in self.sensor_list:
            sensor.destroy()


    def GetState(self):
        self.loc = self.vehicle.get_location()
        vel = self.vehicle.get_velocity()
        self.vel = round(3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2), 0)
        self.acc = self.vehicle.get_acceleration()

    def CreateCamera(self):
        CAMERA_POS_Z = 3
        CAMERA_POS_X = -5

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '360')

        camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z, x=CAMERA_POS_X))
        camera = self.world.spawn_actor(camera_bp, camera_init_trans, attach_to=self.vehicle)
        
        image_w = camera_bp.get_attribute('image_size_x').as_int()
        image_h = camera_bp.get_attribute('image_size_y').as_int()
        self.camera_data = {'image': np.zeros((image_h,image_w,4))}
        camera.listen(lambda image: self.camera_callback(image,self.camera_data))

        cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RGB Camera', self.camera_data['image'])

        self.sensor_list.append(camera)
    def WriteToHUD(self,distance,estimated_throttle):

        image = self.camera_data['image']

        image = cv2.putText(image, 'Speed: ' + str(int(self.vel)) + ' kmh', self.org2,
                            self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)


        image = cv2.putText(image, 'distance: ' + str(int(distance)) + ' meters', self.org3,
                            self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)

   
        image = cv2.putText(image, 'Throttle: ' + str(round(estimated_throttle, 2)) + '', self.org4,
                            self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
        cv2.imshow('RGB Camera', image)

    def CreateVehicles(self):
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()

        vehicle_bp = blueprint_library.filter('*mini*')
        start_point = spawn_points[1]

        target_bp = random.choice(blueprint_library.filter('vehicle.*'))
        target_spawn_point = carla.Transform(carla.Location(x=start_point.location.x - 10,
                                                            y=start_point.location.y,
                                                            z=start_point.location.z),
                                             carla.Rotation(yaw=start_point.rotation.yaw))
        vehicle_spawn_point = carla.Transform(carla.Location(x=start_point.location.x + 10,
                                                             y=start_point.location.y,
                                                             z=start_point.location.z),
                                              carla.Rotation(yaw=start_point.rotation.yaw))

        target_vehicle = self.world.spawn_actor(target_bp, target_spawn_point)
        vehicle = self.world.try_spawn_actor(vehicle_bp[0], vehicle_spawn_point)
        self.object_list.append(target_vehicle)
        self.object_list.append(vehicle)
        self.world.get_spectator().set_transform(start_point)
        return vehicle, target_vehicle

    def run(self):
        self.CleanActors()
        self.client.load_world('Town06')

        self.vehicle, target_vehicle = self.CreateVehicles()

        
        target_vehicle.set_target_velocity(carla.Vector3D(x=-self.target_speed, y=0, z=0))

        self.CreateCamera()

        try:
            start_time = time.time()
            while True:
                current_time = time.time()
                elapsed_time = current_time - start_time

                self.world.tick()
                if cv2.waitKey(1) == ord('q'):
                    break

                self.GetState()
                target_location = target_vehicle.get_location()
                distance = self.loc.distance(target_location)                
                acc_sca = round(-1 * self.acc.x, 2)

                estimated_throttle = self.set_throttle(distance) #control function


                self.WriteToHUD(distance,estimated_throttle)

                self.data_collector.append(elapsed_time, self.vel, estimated_throttle, distance, acc_sca)

        finally:
            self.CleanActors()

if __name__ == "__main__":
    controller = CarlaController()
    controller.run()
    controller.data_collector.PlotData()
