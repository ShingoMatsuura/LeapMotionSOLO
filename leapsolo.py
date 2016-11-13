#####################################################################
# Note: write the below command in terminal to debug
# 
# 1. start drone-kit sitl, --home=[lat],[lon],[alt],[heading]
# dronekit-sitl solo-2.0.20 --home=35.692941,139.776500,0,0
#
# 2. start mavproxy 
# mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
#
# 3. start GCS ( APM Planner etc)
#
#####################################################################

#import Leap
import Leap, sys, thread, time, math
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

#import dronekit / dronekit-sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil
import dronekit_sitl

import common

#global variable
vehicle = None

# altitude to takeoff (m)
ALTITUDE_TO_TAKEOFF = 5.0
# period to takeoff (s)
PERIOD_TO_TAKEOFF = 5.0
PERIOD_TO_RTL = 5.0

# To calculate period
s_time = None

# SOLO states
SOLO_STATE_DISARMED = 0
SOLO_STATE_ARMING = 1
SOLO_STATE_ARMED = 2
SOLO_STATE_IN_AIR = 3
SOLO_STATE_RTL = 4

# SOLO status
SOLO_STATE_NAMES = ['SOLO_STATE_DISARMED', 
					'SOLO_STATE_ARMING', 
					'SOLO_STATE_ARMED', 
					'SOLO_STATE_IN_AIR', 
					'SOLO_STATE_RTL']

SOLO_STATE_IS = SOLO_STATE_DISARMED

# Gesture detect filter
DETECT_CIRCLE_GESTURE_RADIUS = 50.0
DETECT_YAW_DEGREES = 30.0
DETECT_PITCH_DEGREES = 20.0
DETECT_ROLL_DEGREES = 20.0

# move verocity
VEROCITY = 1.0

IS_CONTROLED = False

# 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw


class LeapCustomeListener(Leap.Listener):
#########################################################################
#
# Leap customer listener
#
#########################################################################

	finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
	bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
	state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_STOP']

	def on_init(self, contoller):
		print "Leap was initialized"

	def on_connect(self, contoller):
		print "Leap was connected, waiting for SOLO flight gesture!"

		# enable CIRCLE gesture
		contoller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
		contoller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

	def on_frame(self, contoller):
		global s_time

		frame = contoller.frame()
		
		#print "Frame id: %d" % (frame.id)
		if len(frame.hands) == 0:
			s_time = None
			# if SOLO is in air, set verocity zero to hover
			#solo_force_hover()

		elif len(frame.hands) == 1:
			if not s_time and (is_solo_state(SOLO_STATE_DISARMED) or is_solo_state(SOLO_STATE_IN_AIR)):
				print 'Setted s_time'
				s_time = time.time()
		else:
			s_time = None
	

		if s_time:
			period = time.time() - s_time

			if is_solo_state(SOLO_STATE_DISARMED):
				print 'takeoff after : %.0f(s)' % (PERIOD_TO_TAKEOFF - (period))
			else:
				print 'RTL after : %.0f(s)' % (PERIOD_TO_RTL - (period))

			# Takeoff when two hands are hold for PERIOD_TO_TAKEOFF seconds
			if (period > PERIOD_TO_TAKEOFF) and is_solo_state(SOLO_STATE_DISARMED):
				solo_arm_and_takeoff()
			elif (period > PERIOD_TO_RTL) and is_solo_state(SOLO_STATE_IN_AIR):
				solo_RTL()

		# hands
		for hand in frame.hands:

			# Get the hand's roll, pitch, and yaw
			normal = hand.palm_normal
			direction = hand.direction

			if not is_solo_state(SOLO_STATE_IN_AIR):
				continue

			# left hand
			if hand.is_left:

				# Control Yaw
				if (math.fabs(direction.yaw * Leap.RAD_TO_DEG) > DETECT_YAW_DEGREES):
					if direction.yaw * Leap.RAD_TO_DEG < 0.0:
						print 'turn to LEFT'
						solo_control_yaw(10.0)
					else:
						print 'turn to RIGHT'
						solo_control_yaw(-10.0)

				# Control throttle
				elif (math.fabs(direction.pitch * Leap.RAD_TO_DEG) > DETECT_PITCH_DEGREES):
					if direction.pitch * Leap.RAD_TO_DEG < 0.0:
						print 'go to DOWN'
						solo_throttle(False)
					else:
						print 'go to UP'
						solo_throttle(True)

			# right hand
			else:

				#print "roll: %f degrees, pitch: %f degrees, yaw: %f degrees" % (
				#	direction.roll * Leap.RAD_TO_DEG,
				#	direction.pitch * Leap.RAD_TO_DEG,
				#	direction.yaw * Leap.RAD_TO_DEG)

				# Control roll
				if (math.fabs(direction.yaw * Leap.RAD_TO_DEG) > DETECT_YAW_DEGREES):
					if direction.yaw * Leap.RAD_TO_DEG < 0.0:
						print 'go to LEFT'
						solo_simple_move(0.0, VEROCITY, 0.0)
					else:
						print 'go to RIGHT'
						solo_simple_move(0.0, -VEROCITY, 0.0)

				# Control elevator
				elif (math.fabs(direction.pitch * Leap.RAD_TO_DEG) > DETECT_PITCH_DEGREES):
					if direction.pitch * Leap.RAD_TO_DEG < 0.0:
						print 'go to forward'
						solo_simple_move(VEROCITY, 0.0, 0.0)
					else:
						print 'go to backward'
						solo_simple_move(-VEROCITY, 0.0, 0.0)
						

def solo_force_hover():
#########################################################################
#
# Hover SOLO
#
#########################################################################
	
	global vehicle

	if not is_solo_state(SOLO_STATE_IN_AIR):
		return

	print 'Set SOLO hover!'

	set_solo_controll(True)

	# set velocity 0
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0, 0,
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
			0b0000111111000111,
			0, 0, 0,
			0, 0, 0,
			0, 0, 0,
			0, 0)

	for x in range(0, 1):
		vehicle.send_mavlink(msg)
		time.sleep(1)	

	set_solo_controll(False)


def solo_simple_move(velocity_x, velocity_y, velocity_z, duration = 1):
#########################################################################
#
# Move SOLO by velocity
#
#########################################################################

	global vehicle

	if not is_solo_state(SOLO_STATE_IN_AIR):
		return

	if is_solo_controlled():
		return

	set_solo_controll(True)

	print "Current location %s" % vehicle.location.global_relative_frame

	msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0, 0,
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
			0b0000111111000111,
			0, 0, 0,
			velocity_x, velocity_y, velocity_z,
			0, 0, 0,
			0, 0)

	for x in range(0, duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)

	print "Moved location %s" % vehicle.location.global_relative_frame

	# set velocity 0
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0, 0,
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
			0b0000111111000111,
			0, 0, 0,
			0, 0, 0,
			0, 0, 0,
			0, 0)

	for x in range(0, 1):
		vehicle.send_mavlink(msg)
		time.sleep(1)	

	set_solo_controll(False)


def solo_control_yaw(degrees, duration = 1):
#########################################################################
#
# Set SOLO yaw
#
#########################################################################

	global vehicle

	if not is_solo_state(SOLO_STATE_IN_AIR):
		return

	if is_solo_controlled():
		return

	set_solo_controll(True)

	print 'Leap current Yaw degrees : %f' % (vehicle.heading)

	dir = 0
	if degrees > 0:
		# ccw
		dir = -1
	else:
		# cw
		dir = 1

	msg = vehicle.message_factory.command_long_encode(
		0, 0,
		mavutil.mavlink.MAV_CMD_CONDITION_YAW,
		0,
		math.fabs(degrees),
		0,
		dir,
		1, # 1 is relative, 0 is absolute
		0, 0, 0)

	print 'Leap new Yaw degrees : %f' % (vehicle.heading)

	for x in range(0, duration):
		vehicle.send_mavlink(msg)
		time.sleep(1)

	print 'Leap new Yaw degrees : %f' % (vehicle.heading)
	
	set_solo_controll(False)


def solo_throttle(up):
#########################################################################
#
# Set throttle
#
#########################################################################

	global vehicle

	if not is_solo_state(SOLO_STATE_IN_AIR):
		return	

	if is_solo_controlled():
		return

	set_solo_controll(True)

	original_alt = vehicle.location.global_relative_frame.alt
	new_alt = 0.0

	if up:
		new_alt = original_alt + 1.0
	else:
		new_alt = original_alt - 1.0

	new_point = LocationGlobalRelative(vehicle.location.global_relative_frame.lat,
									vehicle.location.global_relative_frame.lon,
									new_alt)
	vehicle.simple_goto(new_point)

	while True:
		#print 'Altitude: %s' % vehicle.location.global_relative_frame.alt	
		print " %s" % vehicle.location.global_relative_frame

		if up:
			if vehicle.location.global_relative_frame.alt > new_alt * 0.95:
				print 'Reached the target alt!'
				set_solo_controll(False)
				break
		else:
			if vehicle.location.global_relative_frame.alt < new_alt * 1.05:
				print 'Reached the target alt!'
				set_solo_controll(False)
				break				


def solo_arm_and_takeoff(aTargetAltitude = ALTITUDE_TO_TAKEOFF):
#########################################################################
#
# Arming and take off SOLO
#
#########################################################################

	global vehicle

	print 'Start pre-arm checks'
	# Don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print 'Waiting for vehicle to Initialize...'
		time.sleep(1)

	solo_set_state(SOLO_STATE_ARMING)

	print 'Arming motors'
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	print 'Start arming'
	while not vehicle.armed:
		print 'Waiting for arming...'
		time.sleep(1)

	solo_set_state(SOLO_STATE_ARMED)

	print 'Taking off!'
	vehicle.simple_takeoff(aTargetAltitude)

	# Wait until the vehicle reaches a safe height
	while True:
		#print 'Altitude: %s' % vehicle.location.global_relative_frame.alt	
		print " %s" % vehicle.location.global_relative_frame
		if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
			print 'Reached target altitude'
			break;

		time.sleep(1)

	solo_set_state(SOLO_STATE_IN_AIR)


def solo_RTL():
#########################################################################
#
# RTL SOLO
#
#########################################################################

	global vehicle

	solo_set_state(SOLO_STATE_RTL)

	print 'Changed mode to RTL'
	vehicle.mode = VehicleMode("RTL")

	while True:
		#print 'Altitude: %s' % vehicle.location.global_relative_frame.alt
		print " %s" % vehicle.location.global_relative_frame	
		if vehicle.location.global_relative_frame.alt < 0.5:
			print 'Reached ground!'
			break;

		time.sleep(1)

	time.sleep(3)
	solo_set_state(SOLO_STATE_DISARMED)


def print_solo_attributes():
#########################################################################
#
# Print SOLO attributes
#
#########################################################################

	global vehicle

	print "Vehicle state:"
	print " Global Location: %s" % vehicle.location.global_frame
	print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
	print " Local Location: %s" % vehicle.location.local_frame
	print " Attitude: %s" % vehicle.attitude
	print " Velocity: %s" % vehicle.velocity
	print " Battery: %s" % vehicle.battery
	print " Last Heartbeat: %s" % vehicle.last_heartbeat
	print " Heading: %s" % vehicle.heading
	print " Groundspeed: %s" % vehicle.groundspeed
	print " Airspeed: %s" % vehicle.airspeed
	print " Mode: %s" % vehicle.mode.name
	print " Is Armable?: %s" % vehicle.is_armable
	print " Armed: %s" % vehicle.armed


def solo_set_state(aState):
#########################################################################
#
# Set SOLO state
#
#########################################################################

	global SOLO_STATE_IS

	print "Setted SOLO state : %s" % (SOLO_STATE_NAMES[aState])
	SOLO_STATE_IS = aState


def is_solo_state(aState):
#########################################################################
#
# Set Check SOLO
#
#########################################################################

	global SOLO_STATE_IS

	return  True if SOLO_STATE_IS == aState else False


def set_solo_controll(control):
#########################################################################
#
# SOLO controlled state set, true is contrlled by gesture
#
#########################################################################

	global IS_CONTROLED
	IS_CONTROLED = control


def is_solo_controlled():
#########################################################################
#
# Get SOLO controlled state
#
#########################################################################

	global IS_CONTROLED
	return IS_CONTROLED


def main():
#########################################################################
#
# main
#
#########################################################################

	global vehicle

	#
	# to start with dronekit-sitl, write the below in terminal
	#
	#	#python leapsolo.py sitl
	#
	target = sys.argv[1] if len(sys.argv) >= 2 else 'udpin:0.0.0.0:14550'

	is_sitl = False
	if target == 'sitl':
		# comment out. dronekit-stil is started by command line. 
		#sitl = dronekit_sitl.start_default()
		# tcp:127.0.0.1:5760
		#target = sitl.connection_string()
		
		is_sitl = True
		target = 'tcp:127.0.0.1:5762'
	else:
		sitl = None

	print 'Connecting to ' + target + '...'
	
	vehicle = connect(target, wait_ready=True)
	print_solo_attributes()

	#
	# initialize leap
	#
	listener = LeapCustomeListener()
	contoller = Leap.Controller()

	contoller.add_listener(listener)

	print "Press Enter to quit..."

	if is_sitl == True:
		global VEROCITY, ALTITUDE_TO_TAKEOFF
		VEROCITY = 5.0
		ALTITUDE_TO_TAKEOFF = 5.0
		solo_arm_and_takeoff()

	try:
		sys.stdin.readline()
	except KeyboardInterrupt:
		pass
	finally:
		contoller.remove_listener(listener)
		print 'close vehicle'
		vehicle.close()

		if sitl is not None:
			print 'shut down sitl'
			sitl.stop()


#########################################################################
#
# entry point
#
#########################################################################
if __name__ == "__main__":
	main()
