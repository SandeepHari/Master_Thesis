import glob
import os
import sys
import time
import math
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import argparse
import logging
import random
from agents.tools.misc import get_speed
from agents.tools.misc import is_within_distance_ahead
#from multiprocessing import Process
import threading

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)


    try:

        #Load the map and select the world
        world = client.load_world('Town01')
        print(world.get_weather())
        
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)        
        
        
        world.set_weather(weather)
       
        
       
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        rad_ego = None
        lidar_sen = None


        vehicle2_vehicle = None
        vehicle2_cam = None
        vehicle2_col = None
        rad2_vehicle2 = None
        lidar2_sen = None

        vehicle3_vehicle = None
        vehicle3_cam = None
        vehicle3_col = None
        rad3_vehicle3 = None

        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')    
        ego_bp.set_attribute('role_name', 'ego')
        print('\nEgo rolename is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color', ego_color)
        
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        ego_loc = carla.Location(13.568643, 2.1460284, 1.322175)
        ego_rot = carla.Rotation(0.0,0.0,0.0)
        ego_transform = carla.Transform(ego_loc, ego_rot)
        waypoint_ego = world.get_map().get_waypoint(ego_loc, project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        
        ego_loc = waypoint_ego.transform.location
        ego_loc.z += 2
        time.sleep(1.0)
        
        ego_transform = carla.Transform(ego_loc, ego_rot)
        ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
        print('\nEgo is spawned')
        print(ego_transform)

        # Adding camera sensors to the ego vehicle
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x",str(1920))
        cam_bp.set_attribute("image_size_y",str(1080))
        cam_bp.set_attribute("fov",str(105))
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,180,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        ego_cam.listen(lambda image: image.save_to_disk('ACC/ego_camera/%.6d.png' % image.frame))
        print('\nCamera attached')

        #Adding Collision detector to the Ego vehicle(Optionally can add Lane invasion sensor and Obstacle detection)
        col_bp = world.get_blueprint_library().find('sensor.other.collision')
        col_location = carla.Location(0,0,0)
        col_rotation = carla.Rotation(0,0,0)
        col_transform = carla.Transform(col_location,col_rotation)
        ego_col = world.spawn_actor(col_bp,col_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def col_callback(colli):
            print("Collision of ego vehicle detected:\n"+str(colli)+'\n')
        ego_col.listen(lambda colli: col_callback(colli))
        print('\nCollision attached')
        
        # Adding new Lidar sensor to the Ego vehicle
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(32))
        lidar_bp.set_attribute('points_per_second',str(90000))
        lidar_bp.set_attribute('rotation_frequency',str(40))
        lidar_bp.set_attribute('range',str(20))
        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle,attachment_type=carla.AttachmentType.SpringArm)
        lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('ACC/ego_lidar_output/%.6d.ply' % point_cloud.frame))
        print('\nLidar attached')
        
        # Adding new Radar sensor to the ego vehicle
        rad_cam = None
        rad_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad_bp.set_attribute('horizontal_fov', str(35))
        rad_bp.set_attribute('vertical_fov', str(20))
        rad_bp.set_attribute('range', str(20))
        rad_location = carla.Location(x=2.0, z=1.0)
        rad_rotation = carla.Rotation(pitch=5)
        rad_transform = carla.Transform(rad_location,rad_rotation)
        rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def rad_callback(radar_data):
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)
                # The 0.25 adjusts a bit the distance so the dots can
                # be properly seen
                fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

                def clamp(min_v, max_v, value):
                    return max(min_v, min(value, max_v))

                norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                #print("I got here")
                world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
        rad_ego.listen(lambda radar_data: rad_callback(radar_data))
        print('\nRadar attached')
       
        # Making the spectator view relative to the Ego vehicle location
        spectator = world.get_spectator()
        trans = ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(trans.location + carla.Location(z=40), carla.Rotation(pitch = -90)))


        
        # Create the second vehicle
        vehicle2_bp = world.get_blueprint_library().find('vehicle.tesla.model3')    
        vehicle2_bp.set_attribute('role_name', 'vehicle2')
        print('\nVehicle2 rolename is set')
        print('\nVehicle2 rolename is set')

        vehicle2_color = random.choice(vehicle2_bp.get_attribute('color').recommended_values)
        vehicle2_bp.set_attribute('color', vehicle2_color)

        #Spawn the vehicle at the location 10m ahead of ego vehicle
        ego_location = ego_vehicle.get_location()
        ego_rotation = ego_vehicle.get_transform()
        #print(ego_rotation.rotation)
        ego_location.x += 10.0
        #ego_location.z += 2
        print(ego_location)
        waypoint_vehicle2 = world.get_map().get_waypoint(ego_location, project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        ego_location = waypoint_vehicle2.transform.location
        ego_location.z += 2
        time.sleep(1.0)
        vehicle2_transform = carla.Transform(ego_location, ego_rotation.rotation)
        print(vehicle2_transform)
        vehicle2_vehicle = world.spawn_actor(vehicle2_bp, vehicle2_transform)
        print('\nvehicle2 is spawned')
        


        # Adding camera sensors to the second vehicle
        cam2_bp = None
        cam2_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam2_bp.set_attribute("image_size_x",str(1920))
        cam2_bp.set_attribute("image_size_y",str(1080))
        cam2_bp.set_attribute("fov",str(105))
        cam2_location = carla.Location(2,0,1)
        cam2_rotation = carla.Rotation(0,180,0)
        cam2_transform = carla.Transform(cam2_location,cam2_rotation)
        vehicle2_cam = world.spawn_actor(cam2_bp,cam2_transform,attach_to=vehicle2_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        vehicle2_cam.listen(lambda image: image.save_to_disk('ACC/Vehicle2_camera/%.6d.png' % image.frame))
        print('\nvehicle2 camera is Attached')

        #Adding Collision detector to the vehicle2 vehicle(Optionally can add Lane invasion sensor and Obstacle detection)
        col2_bp = world.get_blueprint_library().find('sensor.other.collision')
        col2_location = carla.Location(0,0,0)
        col2_rotation = carla.Rotation(0,0,0)
        col2_transform = carla.Transform(col2_location,col2_rotation)
        vehicle2_col = world.spawn_actor(col2_bp, col2_transform, attach_to=vehicle2_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def col2_callback(colli):
            print("Collision of vehicle2 detected:\n"+str(colli)+'\n')
        vehicle2_col.listen(lambda colli: col2_callback(colli))
        print('\nvehicle2 Collision sensor is Attached')
        
        
        # Adding new Lidar sensor to the vehicle2 vehicle
        lidar2_cam = None
        lidar2_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar2_bp.set_attribute('channels',str(32))
        lidar2_bp.set_attribute('points_per_second',str(90000))
        lidar2_bp.set_attribute('rotation_frequency',str(40))
        lidar2_bp.set_attribute('range',str(20))
        lidar2_location = carla.Location(0,0,2)
        lidar2_rotation = carla.Rotation(0,0,0)
        lidar2_transform = carla.Transform(lidar2_location,lidar2_rotation)
        lidar2_sen = world.spawn_actor(lidar2_bp, lidar2_transform, attach_to=vehicle2_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        lidar2_sen.listen(lambda point_cloud: point_cloud.save_to_disk('ACC/vehicle2_lidar_output/%.6d.ply' % point_cloud.frame))
        print('\nvehicle2 Lidar is Attached')

        # Adding new Radar sensor to the vehicle2 vehicle
        rad2_cam = None
        rad2_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad2_bp.set_attribute('horizontal_fov', str(35))
        rad2_bp.set_attribute('vertical_fov', str(20))
        rad2_bp.set_attribute('range', str(20))
        rad2_location = carla.Location(x=2.0, z=1.0)
        rad2_rotation = carla.Rotation(pitch=5)
        rad2_transform = carla.Transform(rad_location,rad_rotation)
        rad2_vehicle2 = world.spawn_actor(rad_bp,rad_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def rad_callback(radar_data):
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)
                # The 0.25 adjusts a bit the distance so the dots can
                # be properly  while True:
                fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

                def clamp(min_v, max_v, value):
                    return max(min_v, min(value, max_v))

                norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                #print("I got here")
                world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
        rad2_vehicle2.listen(lambda radar_data: rad_callback(radar_data))
        print('\nVehicle2 Radar attached')
        

        # Create the third vehicle
        vehicle3_bp = world.get_blueprint_library().find('vehicle.tesla.model3')    
        vehicle3_bp.set_attribute('role_name', 'vehicle3')
        print('\nVehicle3 rolename is set')
        

        vehicle3_color = random.choice(vehicle3_bp.get_attribute('color').recommended_values)
        vehicle3_bp.set_attribute('color', vehicle3_color)
        
        time.sleep(1.0)
        #Spawn the vehicle at the location 10m ahead of second vehicle
        veh_loc = carla.Location(13.568643, -2.460284, 1.322175)
        veh_rot = carla.Rotation(0.0,0.0,0.0)
        veh_transform = carla.Transform(veh_loc, veh_rot)
        vehicle3_vehicle = world.spawn_actor(vehicle3_bp, veh_transform)
        print('\nvehicle3 is spawned')
        


        # Adding camera sensors to the second vehicle
        cam3_bp = None
        cam3_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam3_bp.set_attribute("image_size_x",str(1920))
        cam3_bp.set_attribute("image_size_y",str(1080))
        cam3_bp.set_attribute("fov",str(105))
        cam3_location = carla.Location(2,0,1)
        cam3_rotation = carla.Rotation(0,180,0)
        cam3_transform = carla.Transform(cam3_location,cam3_rotation)
        vehicle3_cam = world.spawn_actor(cam3_bp,cam3_transform,attach_to=vehicle3_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        vehicle3_cam.listen(lambda image: image.save_to_disk('ACC/vehicle3_camera/%.6d.png' % image.frame))
        print('\nvehicle3 camera is Attached')

        #Adding Collision detector to the vehicle2 vehicle(Optionally can add Lane invasion sensor and Obstacle detection)
        col3_bp = world.get_blueprint_library().find('sensor.other.collision')
        col3_location = carla.Location(0,0,0)
        col3_rotation = carla.Rotation(0,0,0)
        col3_transform = carla.Transform(col3_location,col3_rotation)
        vehicle3_col = world.spawn_actor(col3_bp,col3_transform,attach_to=vehicle3_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def col3_callback(colli):
            print("Collision of vehicle3 detected:\n"+str(colli)+'\n')
        vehicle3_col.listen(lambda colli: col3_callback(colli))
        print('\nvehicle3 Collision sensor is Attached')
        
        
        # Adding new Lidar sensor to the vehicle3
        lidar3_cam = None
        lidar3_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar3_bp.set_attribute('channels',str(32))
        lidar3_bp.set_attribute('points_per_second',str(90000))
        lidar3_bp.set_attribute('rotation_frequency',str(40))
        lidar3_bp.set_attribute('range',str(20))
        lidar3_location = carla.Location(0,0,2)
        lidar3_rotation = carla.Rotation(0,0,0)
        lidar3_transform = carla.Transform(lidar3_location,lidar3_rotation)
        lidar3_sen = world.spawn_actor(lidar3_bp, lidar3_transform, attach_to=vehicle3_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        lidar3_sen.listen(lambda point_cloud: point_cloud.save_to_disk('ACC/vehicle3_lidar_output/%.6d.ply' % point_cloud.frame))
        print('\nvehicle3 Lidar is Attached')

        # Adding new Radar sensor to the vehicle3
        rad3_cam = None
        rad3_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad3_bp.set_attribute('horizontal_fov', str(35))
        rad3_bp.set_attribute('vertical_fov', str(20))
        rad3_bp.set_attribute('range', str(20))
        rad3_location = carla.Location(x=2.0, z=1.0)
        rad3_rotation = carla.Rotation(pitch=5)
        rad3_transform = carla.Transform(rad3_location,rad3_rotation)
        rad3_vehicle3 = world.spawn_actor(rad3_bp, rad3_transform, attach_to=vehicle3_vehicle, attachment_type=carla.AttachmentType.Rigid)
        def rad_callback(radar_data):
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)
                # The 0.25 adjusts a bit the distance so the dots can
                # be properly  while True:
                fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

                def clamp(min_v, max_v, value):
                    return max(min_v, min(value, max_v))

                norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                #print("I got here")
                world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
        rad3_vehicle3.listen(lambda radar_data: rad_callback(radar_data))
        print('\nVehicle3 Radar attached')
        
        
        
        #Set ego vehicle in autopilot
        #ego_vehicle.set_autopilot(True)
        #Set ego vehicle in autopilot
        #vehicle2_vehicle.set_autopilot(True)

        '''
        ### ADDITIONAL CODE FOR VEHICLE PHYSICS CONTROL

        # Create Wheels Physics Control
        front_left_wheel  = carla.WheelPhysicsControl(tire_friction=4.5, damping_rate=1.0, max_steer_angle=70.0, radius=30.0)
        front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.5, damping_rate=1.5, max_steer_angle=70.0, radius=25.0)
        rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=1.0, damping_rate=0.2, max_steer_angle=0.0,  radius=15.0)
        rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=1.5, damping_rate=1.3, max_steer_angle=0.0,  radius=20.0)

        wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

        # Change Vehicle Physics Control parameters of the vehicle
        physics_control = vehicle.get_physics_control()

        physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        physics_control.max_rpm = 10000
        physics_control.moi = 1.0
        physics_control.damping_rate_full_throttle = 0.0
        physics_control.use_gear_autobox = True
        physics_control.gear_switch_time = 0.5  
        physics_control.clutch_strength = 10
        physics_control.mass = 10000
        physics_control.drag_coefficient = 0.25
        physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
        physics_control.wheels = wheels

        # Apply Vehicle Physics Control for the vehicle
        ego_vehicle.apply_physics_control(physics_control)

        '''
        
        #CreateScenario1()
        # Initial condition
        # V1(20kmh) -----10m----- V2(20kmh)
        # Step1: Drive 30 seconds 
        # Step2: V2 decelerates to 10kmh with 2m/s2
        # Expected behavior: V1 slows down to 10 with keeping (ACC distance - 10?) 
    
        # Making the spectator view relative to the Ego vehicle location
        #spectator = world.get_spectator()
        #trans = vehicle2_vehicle.get_transform()
        #spectator.set_transform(carla.Transform(trans.location))
        
    
        #Creating Scenario 1
        actors = world.get_actors()
        vehicle3_vehicle.apply_control(carla.VehicleControl(manual_gear_shift = True, gear=1))
        vehicle2_vehicle.apply_control(carla.VehicleControl(manual_gear_shift = True, gear=1))
        ego_vehicle.apply_control(carla.VehicleControl(manual_gear_shift = True, gear=1))
        vehicle3_vehicle.apply_control(carla.VehicleControl(manual_gear_shift = False))
        vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = False))
        ego_vehicle.apply_control(carla.VehicleControl(throttle = False))


        vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.8))
        vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.7))
        ego_vehicle.apply_control(carla.VehicleControl(throttle = 0.7))        
        time.sleep(5.0)
        v3_velocity = get_speed(vehicle3_vehicle)
        print(v3_velocity)
        v2_velocity = get_speed(vehicle2_vehicle)
        print(v2_velocity)
        ego_velocity = get_speed(ego_vehicle)
        print(ego_velocity)
        

        #Perform CutIn
        
        while True:
            v3_pos = vehicle3_vehicle.get_transform()
            v2_pos = vehicle2_vehicle.get_transform()
            sep = v3_pos.location.x - v2_pos.location.x
            cond = sep < 7.0
            #dist_check = is_within_distance_ahead(vehicle3_vehicle.get_transform(), vehicle2_vehicle.get_transform(), 10.0)
            if not cond:
                #vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.6, steer = 0.05))
                #time.sleep(0.5)
                #print("First steer")
                #vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.6, steer = -0.13))
                #print("Second steer")
                #vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.6, steer = -0.10))
                #print("Third steer")
                #time.sleep(0.5)
                #vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.6, steer = 0.00))
                #print("Fourth steer") 

                #time.sleep(5.0)
                v3_pos = vehicle3_vehicle.get_transform()
                v3_pos.location.x+= 3.0
                v3_pos.location.y = 2.1460284
                vehicle3_vehicle.set_transform(v3_pos)

                break
                
        
        vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.3))

        dist_check = is_within_distance_ahead(vehicle3_vehicle.get_transform(), vehicle2_vehicle.get_transform(), 13.0)
        if dist_check :        
            vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.1))
            time.sleep(2.0)
        
        while True:
            
            dist_check = is_within_distance_ahead(vehicle2_vehicle.get_transform(), ego_vehicle.get_transform(), 13.0)
            while dist_check:
                ego_vehicle.apply_control(carla.VehicleControl(throttle = 0.2))
                vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.5))
                #time.sleep(2.0)
                dist_check = is_within_distance_ahead(vehicle2_vehicle.get_transform(), ego_vehicle.get_transform(), 13.0)
                if not dist_check : 
                    ego_vehicle.apply_control(carla.VehicleControl(throttle = 0.5))  
            
            first_dist = is_within_distance_ahead(vehicle3_vehicle.get_transform(), vehicle2_vehicle.get_transform(), 13.0)
            if not dist_check :
                print("Entered")
                vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.5)) 
                vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.3))
                vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.4))
            else :
                print("Twice Entered")
                vehicle2_vehicle.apply_control(carla.VehicleControl(throttle = 0.1))
                vehicle3_vehicle.apply_control(carla.VehicleControl(throttle = 0.5))


        
        
        while True:
           world_snapshot = world.wait_for_tick()

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if rad_ego is not None:
                rad_ego.stop()
                rad_ego.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
            ego_vehicle.destroy()
        
        if vehicle2_vehicle is not None:
            if vehicle2_cam is not None:
                vehicle2_cam.stop()
                vehicle2_cam.destroy()
            if vehicle2_col is not None:
                vehicle2_col.stop()
                vehicle2_col.destroy()
            if rad2_vehicle2 is not None:
                rad2_vehicle2.stop()
                rad2_vehicle2.destroy()
            if lidar2_sen is not None:
                lidar2_sen.stop()
                lidar2_sen.destroy()
            vehicle2_vehicle.destroy()
        
        if vehicle3_vehicle is not None:
            if vehicle3_cam is not None:
                vehicle3_cam.stop()
                vehicle3_cam.destroy()
            if vehicle3_col is not None:
                vehicle3_col.stop()
                vehicle3_col.destroy()
            if rad3_vehicle3 is not None:
                rad3_vehicle3.stop()
                rad3_vehicle3.destroy()
            if lidar3_sen is not None:
                lidar3_sen.stop()
                lidar3_sen.destroy()
            vehicle3_vehicle.destroy()
        
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with the scenario.')
