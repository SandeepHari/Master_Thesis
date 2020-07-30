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
import threading
# Connects to the simulator instance at the ip defined by SIMULATOR_HOST, default is localhost or 127.0.0.1
sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)

print("Version =", sim.version)
print("Current Time =", sim.current_time)
print("Current Frame =", sim.current_frame)

#Set the ACC dist
ACC_DIST = 15.0




#Load the Map. Selecting the single lane road
if sim.current_scene == "BorregasAve":
    sim.reset()
else:
    sim.load(scene = "BorregasAve", seed = 650387)

print("Current Scene = {}".format(sim.current_scene))

sim.weather = lgsvl.WeatherState(rain=0.8, fog=0.3, wetness=0)
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


#Spawn the third vehicle
third_state = ego_2.state
third_state.transform = ego_2.state.transform
third_state.transform.position = lgsvl.Vector(-6.309202262733223, -1.03600001335144, 45.50)
third_state.transform.rotation = lgsvl.Vector(0, 194.823394775391, 0)
print(third_state.transform)
ego_3 = sim.add_agent("Lincoln2017MKZ (Apollo 5.0)", lgsvl.AgentType.EGO, third_state)


#Set the speed of the ego vehicle
s_vehicle1 = ego_2.state
s_vehicle1.velocity = 8 * forward
ego_2.state = s_vehicle1

#Set the speed of the first vehicle
s_ego = ego_1.state
s_ego.velocity = 8 * forward
ego_1.state = s_ego

#Set the speed of the third vehicle
s_vehicle2 = ego_3.state
s_vehicle2.velocity = 13 * forward
ego_3.state = s_vehicle2



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

#Get the sensor data from the cutin vehicle
for sensor in ego_3.get_sensors():
	print(sensor.name, sensor.enabled)
	if sensor.name == "Main Camera":
        	sensor.save("main-camera_lead.png", compression=0)
	if sensor.name == "Lidar":
        	sensor.save("lidar_lead.pcd")
	if sensor.name == "GPS":
		gps_sensor_cutin= sensor



sim.run(time_limit = 5.0)



# Get the gps sensor data to obtain location
gps_lead = gps_sensor_lead.data
gps_ego = gps_sensor_ego.data

#Function to calculate the distance between two vehicles	
def main_dist_check(gps_cutin, gps_lead):
	tr1 = sim.map_from_gps(gps_cutin.latitude, gps_cutin.longitude)
	tr2 = sim.map_from_gps(gps_lead.latitude, gps_lead.longitude)
	diff = tr1.position.z - tr2.position.z
	#print("gap:", diff)
	return diff 


def dist_check(gps_lead, gps_ego):
	tr1 = sim.map_from_gps(gps_lead.latitude, gps_lead.longitude)
	tr2 = sim.map_from_gps(gps_ego.latitude, gps_ego.longitude)
	diff = tr1.position.z - tr2.position.z
	#print("gap:", diff)
	return diff 

#Initial distance check
dist = dist_check(gps_lead, gps_ego)
#print("Distance:", dist)


#Perform Cut-In in front of the lead vehicle
st = lgsvl.VehicleControl()
st.steering = 0.1
ego_3.apply_control(st, True)
sim.run(1.0)


st = lgsvl.VehicleControl()
st.steering = -0.1
ego_3.apply_control(st, True)
sim.run(1.0)

st = lgsvl.VehicleControl()
st.steering = 0.0
ego_3.apply_control(st, True)

# Trigger for vehicles in ACC platoon
s_vehicle2 = ego_3.state
s_vehicle2.velocity = 6 * forward
ego_3.state = s_vehicle2

st = lgsvl.VehicleControl()
st.steering = -0.01
ego_3.apply_control(st, True)
sim.run(1.0)

st = lgsvl.VehicleControl()
st.steering = 0.01
ego_3.apply_control(st, True)
#sim.run(0.5)

print("Steered")

st = lgsvl.VehicleControl()
st.steering = 0.00
ego_3.apply_control(st, True)
sim.run(1.0)

s_vehicle2 = ego_3.state
s_vehicle2.velocity = 10 * forward
ego_3.state = s_vehicle2

#Enabling the ACC and also braking if the cut-in happens close to the lead vehicle

gps_cutin = gps_sensor_cutin.data
gps_lead = gps_sensor_lead.data
dist_main = main_dist_check(gps_cutin, gps_lead)

if dist_main > -15.0:
	s_vehicle1 = ego_2.state
	s_vehicle1.velocity = 3 * forward
	ego_2.state = s_vehicle1
	print("Brake applied")

	#check whether the distance is within ACC range
	if dist > -15.0 :
		print("Entered")
		s_ego = ego_1.state
		s_ego.velocity = 2 * forward
		ego_1.state = s_ego
		gps_lead = gps_sensor_lead.data
		gps_ego = gps_sensor_ego.data
		dist = dist_check(gps_lead, gps_ego)



	gps_cutin = gps_sensor_cutin.data
	gps_lead = gps_sensor_lead.data
	dist_main = main_dist_check(gps_cutin, gps_lead)
	
sim.run(time_limit = 8.0)



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
			s.throttle = 0.3
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

	s_vehicle2 = ego_3.state
	s_vehicle2.velocity = 10 * forward
	ego_3.state = s_vehicle2

	s = lgsvl.VehicleControl()
	s.throttle = 0.1
	ego_2.apply_control(s, True)
	
	a = lgsvl.VehicleControl()
	a.throttle = 0.15
	ego_1.apply_control(a, True)	
	sim.run(time_limit = 2.0)


# Run the Simulation indefinitely
sim.run()





