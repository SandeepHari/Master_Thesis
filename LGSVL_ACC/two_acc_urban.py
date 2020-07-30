#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl
import math
import random
import time

# Connects to the simulator instance at the ip defined by SIMULATOR_HOST, default is localhost or 127.0.0.1
sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)

print("Version =", sim.version)
print("Current Time =", sim.current_time)
print("Current Frame =", sim.current_frame)


#Load the Map. Selecting the Borregas Avenue Map
if sim.current_scene == "BorregasAve":
    sim.reset()
else:
    sim.load(scene = "BorregasAve", seed = 650387)

print("Current Scene = {}".format(sim.current_scene))

#Setting the weather conditions. 
sim.weather = lgsvl.WeatherState(rain=0.8, fog=0, wetness=0)
print(sim.weather)


#Spawn the EGO Vehicle
spawns = sim.get_spawn()
state = lgsvl.AgentState()
state.transform = spawns[0]
forward = lgsvl.utils.transform_to_forward(spawns[0])
up = lgsvl.utils.transform_to_up(spawns[0])
state.transform.position += 5 * forward# 5m forwards
ego_1 = sim.add_agent("Lincoln2017MKZ (Apollo 5.0)", lgsvl.AgentType.EGO, state)
print(state.transform)



#Spawn the second vehicle
ego_state = ego_1.state
ego_state.transform = ego_1.state.transform
ego_state.transform.position += 15 * forward
print(ego_state.transform)
ego_2 = sim.add_agent("Jaguar2015XE", lgsvl.AgentType.EGO, ego_state)

# Give the vehicles Initial Velocity
s_vehicle1 = ego_2.state
s_vehicle1.velocity = 8 * forward
ego_2.state = s_vehicle1

s_ego = ego_1.state
s_ego.velocity = 8 * forward
ego_1.state = s_ego


#Get the sensor data from the lead vehicle
for sensor in ego_2.get_sensors():
	print(sensor.name, sensor.enabled)
	if sensor.name == "Main Camera":
        	sensor.save("main-camera_lead.png", compression=0)
	if sensor.name == "Lidar":
        	sensor.save("lidar_lead.pcd")
	if sensor.name == "GPS":
		gps_sensor_lead = sensor


#Get the sensor data from the ego vehicle
for sensor in ego_1.get_sensors():
	print(sensor.name, sensor.enabled)
	if sensor.name == "Main Camera":
        	sensor.save("main-camera_ego.png", compression=0)
	if sensor.name == "Lidar":
        	sensor.save("lidar_ego.pcd")
	if sensor.name == "GPS":
		gps_sensor_ego = sensor

sim.run(time_limit = 5.0)



# Get the gps sensor data to obtain location
gps_lead = gps_sensor_lead.data
gps_ego = gps_sensor_ego.data


#Function to calculate the distance between two vehicles	
def dist_check(gps_lead, gps_ego):
	tr1 = sim.map_from_gps(gps_lead.latitude, gps_lead.longitude)
	tr2 = sim.map_from_gps(gps_ego.latitude, gps_ego.longitude)
	diff = tr1.position.z - tr2.position.z
	#print("gap:", diff)
	return diff 

#Initial distance check
dist = dist_check(gps_lead, gps_ego)
#print("Distance:", dist)

#Trigger condition - Vehicle 1 slows
s = lgsvl.VehicleControl()
s.brake = 0.9
ego_2.apply_control(s, True)
print("Brake applied")
sim.run(time_limit = 6.0)

#Continuos ACC update and slowing down of the vehicles
while True :
	gps_lead = gps_sensor_lead.data
	gps_ego = gps_sensor_ego.data
	dist = dist_check(gps_lead, gps_ego)
	print("First dist check done")

	#check whether the distance is within ACC range
	while dist > -15.0 :
			print("Entered")
			c = lgsvl.VehicleControl()
			c.brake = 0.9
			ego_1.apply_control(c, True)
			sim.run(time_limit = 5.0)
			print("Second brake applied")

			#Continuous trigger conditions
			s = lgsvl.VehicleControl()
			s.throttle = 0.2
			ego_2.apply_control(s, True)
			a = lgsvl.VehicleControl()
			a.throttle = 0.1
			ego_1.apply_control(a, False)
			sim.run(time_limit = 2.0)

			s = lgsvl.VehicleControl()
			s.brake = 0.3
			ego_2.apply_control(s, True)

			a = lgsvl.VehicleControl()
			a.throttle = 0.1
			ego_1.apply_control(a, True)	
			sim.run(time_limit = 2.0)

			gps_lead = gps_sensor_lead.data
			gps_ego = gps_sensor_ego.data
			dist = dist_check(gps_lead, gps_ego)

	s = lgsvl.VehicleControl()
	s.throttle = 0.1
	ego_2.apply_control(s, True)
	
	a = lgsvl.VehicleControl()
	a.throttle = 0.15
	ego_1.apply_control(a, True)	
	sim.run(time_limit = 2.0)


# Run the Simulation indefinitely
sim.run()


















