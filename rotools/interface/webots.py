# This example use UR10e robot simulated in Webots

"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()
ur5_motor_shoulder_pan_joint = Motor('shoulder_pan_joint')

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
pos = 0.
forward = True
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    pos = pos + 0.1 if forward else pos - 0.1
    ur5_motor_shoulder_pan_joint.setPosition(pos)
    ur5_motor_shoulder_pan_joint.setVelocity(3)  # maxvel = 3.14
    if pos >= 3.0:
        forward = False
    if pos <= -3.0:
        forward = True
    pass

# Enter here exit cleanup code.
