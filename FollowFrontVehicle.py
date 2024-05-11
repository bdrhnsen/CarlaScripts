import glob
import math
import os
import sys
try:
    sys.path.append('C:/Users/acer/Downloads/CARLA_0.9.15/WindowsNoEditor/PythonAPI/carla') 
except IndexError:
    pass
import carla

import random
import time
import cv2 
import matplotlib.pyplot as plt
import numpy as np
def SetThrottle(distance,prev_error,err_sum):
    dt = 1/20.0
    MAX_ERR = 50
    WANTED_DISTANCE = 40

    KP = 1 
    KD=0
    KI =0
    
    err =distance - WANTED_DISTANCE

    derivative = (err-prev_error)/dt
    err_sum += err*dt

    throttle = KP*err + KD*derivative + KI* err_sum
    '''
    Returns throttle that will be applied to the vehicle
    in carla 1.0 throttle is the maximum
    '''
    
    return err,err_sum,max(0,min(1,throttle/MAX_ERR))  



#adding params to display text to image
font = cv2.FONT_HERSHEY_SIMPLEX
# org - defining lines to display telemetry values on the screen
org = (30, 30) # this line will be used to show current speed
org2 = (30, 50) # this line will be used for future steering angle
org3 = (30, 70) # and another line for future telemetry outputs
org4 = (30, 90) # and another line for future telemetry outputs
org3 = (30, 110) # and another line for future telemetry outputs
fontScale = 0.5
# white color
color = (255, 255, 255)
# Line thickness of 2 px
thickness = 1

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
client.load_world('Town06')
# Once we have a client we can retrieve the world that is currently
# running.
world = client.get_world()

for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()

blueprint_library = world.get_blueprint_library()

spawn_points = world.get_map().get_spawn_points()
#look for a blueprint of Mini car
vehicle_bp = blueprint_library.filter('*mini*')

start_point = spawn_points[1]
target_bp = random.choice(blueprint_library.filter('vehicle.*'))

target_spawn_point = carla.Transform(carla.Location(x=start_point.location.x-10, y=start_point.location.y, z=start_point.location.z), carla.Rotation(yaw=start_point.rotation.yaw))
vehicle_spawn_point = carla.Transform(carla.Location(x=start_point.location.x+10, y=start_point.location.y, z=start_point.location.z), carla.Rotation(yaw=start_point.rotation.yaw))

target_vehicle = world.spawn_actor(target_bp, target_spawn_point)
vehicle = world.try_spawn_actor(vehicle_bp[0], vehicle_spawn_point)

target_speed = 50.0/3.6


target_vehicle.set_target_velocity(carla.Vector3D(x=-target_speed, y=0, z=0))


#vehicle.set_target_velocity(carla.Vector3D(x=-target_speed, y=0, z=0))

#camera mount offset on the car - you can tweak these to have the car in view or not
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
camera_bp.set_attribute('image_size_y', '360')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
#this creates the camera in the sim
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# this actually opens a live stream from the camera
camera.listen(lambda image: camera_callback(image,camera_data))

cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera',camera_data['image'])
world.get_spectator().set_transform(start_point)
prev_error = 0
sum_error = 0
time_list = []
speed_list = []
throttle_list = []
distance_list = []
acc_list = []
try:
    start_time = time.time()
    while True:

        current_time = time.time()

        # Calculate elapsed time
        elapsed_time = current_time - start_time

        world.tick() 
        if cv2.waitKey(1) == ord('q'):
            quit = True
            break
        image = camera_data['image']
        steering_angle = 0 
        loc = vehicle.get_location()
        vel = vehicle.get_velocity()
        acc = vehicle.get_acceleration()
        acc_sca = round(-1*acc.x,2)
        #print(f"accelartion is:  {acc_sca}")
        speed = round(3.6 * math.sqrt(vel.x**2 + vel.y**2),0)
        # now we add the speed to the window showing a camera mounted on the car
        image = cv2.putText(image, 'Speed: '+str(int(speed))+' kmh', org2, 
                            font, fontScale, color, thickness, cv2.LINE_AA)
        

        target_location = target_vehicle.get_location()

        # Calculate distance between ego and target vehicles
        distance = loc.distance(target_location)

        image = cv2.putText(image, 'distance: '+str(int(distance))+' meters', org3, 
                    font, fontScale, color, thickness, cv2.LINE_AA)


        prev_error,sum_error, estimated_throttle = SetThrottle(distance, prev_error,sum_error)
        # now we apply accelerator
        vehicle.apply_control(carla.VehicleControl(throttle=estimated_throttle, 
                                                    steer=steering_angle))
        image = cv2.putText(image, 'Throttle: '+str(round(estimated_throttle,2))+'', org4, 
                    font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.imshow('RGB Camera',image)

        time_list.append(elapsed_time)
        speed_list.append(speed)
        throttle_list.append(estimated_throttle)
        distance_list.append(distance)
        acc_list.append(acc_sca)
finally:
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for sensor in world.get_actors().filter('*sensor*'):
        sensor.destroy()


plt.plot(time_list, speed_list)
plt.xlabel('Time (s)')
plt.ylabel('Ego Vehicle Speed (m/s)')
plt.title('Ego Vehicle Speed as a Function of Time')
plt.grid(True)

plt.figure()
plt.plot(time_list, distance_list)
plt.xlabel('Time (s)')
plt.ylabel('Distance between ego and target')
plt.title('Distance between ego and target as a Function of Time')
plt.grid(True)

plt.figure()
plt.plot(time_list, acc_list)
plt.xlabel('Time (s)')
plt.ylabel('Acc of ego')
plt.title('Acc of ego  as a Function of Time')
plt.grid(True)


plt.figure()
plt.plot(time_list, throttle_list)
plt.xlabel('Time (s)')
plt.ylabel('throttle ego')
plt.title('throttle as a Function of Time')
plt.grid(True)


plt.show()