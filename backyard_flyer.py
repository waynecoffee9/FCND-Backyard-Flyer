import argparse
import time
from enum import Enum
import math
import numpy as np
import visdom

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
	MANUAL = 0
	ARMING = 1
	TAKEOFF = 2
	WAYPOINT = 3
	LANDING = 4
	DISARMING = 5


class BackyardFlyer(Drone):

	def __init__(self, connection):
		super().__init__(connection)
		self.target_position = np.array([0.0, 0.0, 0.0])
		self.all_waypoints = []
		self.in_mission = True
		self.check_state = {}
		self.waypoint_idx = -1
		# initial state
		self.flight_state = States.MANUAL
		## real time logging
		'''
		# default opens up to http://localhost:8097
		self.v = visdom.Visdom()
		assert self.v.check_connection()
		
		# Plot NE
		ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
		self.ne_plot = self.v.scatter(ne, opts=dict(
			title="Local position (north, east)", 
			xlabel='North', 
			ylabel='East'
		))

		# Plot D
		d = np.array([self.local_position[2]])
		self.t = 0
		self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
			title="Altitude (meters)", 
			xlabel='Timestep', 
			ylabel='Down'
		))
		self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
		self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)
		'''
		# TODO: Register all your callbacks here
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)
	
	# for real time logging
	def update_ne_plot(self):
		ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
		self.v.scatter(ne, win=self.ne_plot, update='append')
		
	# for real time logging
	def update_d_plot(self):
		d = np.array([self.local_position[2]])
		# update timestep
		self.t += 1
		self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')
		
	def distance(self, pt1, pt2):
		return ((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2 + (-pt1[2]-pt2[2])**2)**0.5
		
	def local_position_callback(self):
		"""
		TODO: Implement this method

		This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
		"""
		print("flight state: ", self.flight_state)
		altitude = -1.0 * self.local_position[2]
		# when drone is close to target altitude, switch to waypoint state
		if self.flight_state == States.TAKEOFF:
			if altitude > 0.95 * self.target_position[2]:
				self.waypoint_transition()
		# check if drone is close to each target waypoint with local position and velocity
		elif self.flight_state == States.WAYPOINT:
			dist = self.distance(self.target_position, self.local_position)
			vel = self.distance([0.0,0.0,0.0], self.local_velocity)
			print("current position: ", self.local_position)
			print("target position: ", self.target_position)
			if dist < 0.3 and vel < 0.3:
				# if last waypoint is reached, switch to landing state
				if self.waypoint_idx == len(self.all_waypoints) - 1:
					self.landing_transition()
				else: self.waypoint_transition()
				
	def velocity_callback(self):
		"""
		TODO: Implement this method

		This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
		"""
		if self.flight_state == States.LANDING:
			if ((self.global_position[2] - self.global_home[2] < 0.1) and
			abs(self.local_position[2]) < 0.05):
				self.disarming_transition()

	def state_callback(self):
		"""
		TODO: Implement this method

		This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
		"""
		if not self.in_mission:
			return
		if self.flight_state == States.MANUAL:
			self.arming_transition()
		elif self.flight_state == States.ARMING:
			if self.armed:
				self.takeoff_transition()
		elif self.flight_state == States.DISARMING:
			if not self.armed:
				self.manual_transition()

	def calculate_box(self):
		"""TODO: Fill out this method
		
		1. Return waypoints to fly a box
		"""
		return [[10.0,0.0,3.0],[10.0,10.0,3.0],[0.0,10.0,3.0],[0.0,0.0,3.0]]

	def arming_transition(self):
		"""TODO: Fill out this method
		
		1. Take control of the drone
		2. Pass an arming command
		3. Set the home location to current position
		4. Transition to the ARMING state
		"""
		self.take_control()
		self.arm()

		self.all_waypoints = self.calculate_box()
		
		
		# set the current location to be the home position
		self.set_home_position(self.global_position[0],
							   self.global_position[1],
							   self.global_position[2])

		self.flight_state = States.ARMING
		print("arming transition")

	def takeoff_transition(self):
		"""TODO: Fill out this method
		
		1. Set target_position altitude to 3.0m
		2. Command a takeoff to 3.0m
		3. Transition to the TAKEOFF state
		"""
		print("takeoff transition")
		target_altitude = 3.0
		self.target_position[2] = target_altitude
		self.takeoff(target_altitude)
		self.flight_state = States.TAKEOFF
		print("takeoff transition")

	def waypoint_transition(self):
		"""TODO: Fill out this method
	
		1. Command the next waypoint position
		2. Transition to WAYPOINT state
		"""
		self.waypoint_idx += 1;
		self.target_position = self.all_waypoints[self.waypoint_idx]
		self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
		self.flight_state = States.WAYPOINT
		print("waypoint transition")

	def landing_transition(self):
		"""TODO: Fill out this method
		
		1. Command the drone to land
		2. Transition to the LANDING state
		"""
		print("landing transition")
		self.land()
		self.flight_state = States.LANDING
		print("landing transition")

	def disarming_transition(self):
		"""TODO: Fill out this method
		
		1. Command the drone to disarm
		2. Transition to the DISARMING state
		"""
		print("disarm transition")
		self.disarm()
		self.flight_state = States.DISARMING
		print("disarm transition")

	def manual_transition(self):
		"""This method is provided
		
		1. Release control of the drone
		2. Stop the connection (and telemetry log)
		3. End the mission
		4. Transition to the MANUAL state
		"""
		print("manual transition")

		self.release_control()
		self.stop()
		self.in_mission = False
		self.flight_state = States.MANUAL

	def start(self):
		"""This method is provided
		
		1. Open a log file
		2. Start the drone connection
		3. Close the log file
		"""
		print("Creating log file")
		self.start_log("Logs", "NavLog.txt")
		print("starting connection")
		self.connection.start()
		print("Closing log file")
		self.stop_log()


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--port', type=int, default=5760, help='Port number')
	parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
	args = parser.parse_args()

	conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
	#conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
	drone = BackyardFlyer(conn)
	time.sleep(2)
	drone.start()
