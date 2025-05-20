[comment]: render

# Final Exam Challenge: Autonomous Robotics Group Project

## Objective:

In randomized groups, students will program their ROMI robots to autonomously complete a series of tasks: moving in a straight line, turning, and detecting a bump using the accelerometer.


### Task Description:

- Move 20 inches in a straight line
- Turn 90 Degrees counter-clockwise
- Move 15 inches in a straight line
- Turn 45 degrees clockwise
- Move until the robot detects that it is on top of a platform. 
- The robot should stop after it starts to level out.
- Robot is fully autonomous

#### Video Example

https://youtube.com/shorts/P69p2e3BHAM?feature=share

#### Starter Repo

https://github.com/mbardoeChoate/ROMI-Characterizable.git

## Requirements:

- PID Controller: Use PID controllers for precise movement and turning.
- Commands2 Library: Structure the task using the commands2 library.
- Accelerometer Integration: Use the accelerometer to detect the end of the ramp.
- Group Collaboration: Groups will be randomized, and each group must collaborate to complete the task.

### Example Code Snippet:

The following code snippets provide an example of how to structure the autonomous commands using the commands2 library and PID controllers.
They won't work, and require some modifications. But they may be helpful in understanding the basic structure.

#### arcadeDrive function that integrates Feedforward and PID

```python
    def arcadeDrive(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        # 1. Get arcade drive input
        fwd = -fwd * self.MAX_LINEAR_SPEED  # forward
        rot = rot * self.MAX_ANGULAR_SPEED  # rotation

        # 2. Convert to chassis speeds
        chassis_speeds = ChassisSpeeds(fwd, 0.0, rot)
        # use this chassis speed to set it specifically for characterization
        # chassis_speeds = ChassisSpeeds(4, 0.0, 0.0)

        # 3. Convert chassis speeds to wheel speeds
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis_speeds)

        # 4. PID
        # in rotations per second
        actual_left_speed = self.leftEncoder.getRate() * self.kWheelDiameterInch * .0254 * math.pi  
        actual_right_speed = self.rightEncoder.getRate() * self.kWheelDiameterInch * .0254 * math.pi

        self.left_speed = wheel_speeds.left / (self.kWheelDiameterInch * .0254 * math.pi)  # Rotations per second
        self.right_speed = wheel_speeds.right / (self.kWheelDiameterInch * .0254 * math.pi)

        # 5. Add compute correction from a PIDController
        left_pid = self.pid_left.calculate(actual_left_speed, self.left_speed)
        right_pid = self.pid_right.calculate(actual_right_speed, self.right_speed)

        # 6. Use feedforward to get voltages
        left_voltage = self.feedforward_left.calculate(self.left_speed)
        right_voltage = self.feedforward_right.calculate(self.right_speed)

        # 5. Send voltages to motors
        self.leftMotor.setVoltage((left_voltage + left_pid))
        self.rightMotor.setVoltage(-(right_voltage + right_pid))
```

##### Tuning the FeedForward and the PID

Finding the feedforward constants (sometimes referred to as *Characterization*) is an important way to make your 
autonomous commands more consistent. For what we are doing it is primarily important to determine $k_S$ and $k_V$. 

###### Finding $k_S$

$k_S$ is the amount of voltage necessary to get the robot motors to start to move. Start this process by setting the 
desired speed to some very small amount, and setting $k_V$ and $k_A$ to zero. Then through trial and error find the 
$k_S$ that make the wheels start to spin when on the ground. Work to make the $k_S$ as small as possible, but still 
gets the robot to move.

###### Finding $k_V$

Set the `ChassisSpeed` of the robot manually to a specific setting, then guess a value for $K_V$ (2 point something 
wouldn't be unreasonable). Run the robot, and graph the difference between the set speed, and the actual speed as 
measured by the encoders. You want these to match. It can be helpful to mark which motor is left and right, use the 
tendency of the robot to drift to one side to modify the values so that when going forward the robot drives straight.
(I did have difficulty getting the robot to run straight going both forward and backwards. Luckily, our task doesn't 
require backward movement.)

###### Finding PID Values

Once the feedforward is working. You will want to tune the PID. These values can be tuned by zeroing out the 
contribution of the feedforward, then getting the PID do match requested speeds in a way that is similar to the 
what you did for the feedforward. 

###### Putting it together

Once you have tuned both the feedforward and the PID you will want to test them together. It may be that together you 
get more velocity that you want, and if that is the case, then lower your constants for the PID controller, to make it
less aggressive.


#### Optional: An alternative PID Controller

In the drive straight command, sometimes the distance calculations are off because the PIDController for the distance 
asks for so much velocity right at the start. This causes the wheels to lose traction and the encoder counts turns that 
don't translate to real distance. One way around this is with a `ProfiledPIDController`. A profiled PIDController put 
maximums on the velocity and acceleration that will be asked from the PIDController. This can allow more control at the 
start. ProfiledPIDControllers work similarly to PIDControlllers but you must add in constraints at the constructor. 
Those constraints are of the `TrapezoidalProfle.Constraints` class. The first number is the maximum speed, and the 
second is maximum acceleration.

```python
        constraints = TrapezoidProfile.Constraints(1*100/2.54, 0.5*100/2.54)
        self.forward_controller = ProfiledPIDController(.4, 0, 0.01, constraints=constraints)
        
        # Then use the setGoal, setTolerance, calculate and atGoal methods  
```

#### Main Autonomous Command Sequence:

This is another way to create a complex command. This is a SequentialCommandGroup that runs the commands in sequence.
Then in `robot.py` you can import and then schedule this command in the `autonomousInit` method. 

This allows us to work out each step of the process individually, then put it all together once each step works very 
well. Sometimes our commands have difficulty coming to definitive end, because the tolerance may be too stringent. So 
it is good to add a timeout to each step of the process. 

```python

import commands2

class AutonomousSequence(commands2.SequentialCommandGroup):
    def __init__(self, drivetrain, accelerometer):
        super().__init__(
            # Move 20 inches, put a time limit so that definitely ends.
            MoveStraight(20, drivetrain).withTimeout(3),  
            Turn(90, drivetrain).withTimeout(3),  # Turn 90 degrees to the right
            ## More move straight
            ## The ramp command
        )
```


#### Move a number of Inches Command:

```python

import commands2
from wpilib.controller import PIDController

class MoveStraight(commands2.CommandBase):
    
    def __init__(self, distance, drivetrain):
        super().__init__()
        ... # Some code goes here
        self.pid = PIDController(0.1, 0, 0.1)

    def initialize(self):
        # reset encoders
        self.drivetrain.reset_encoders()
        # set the setpoint and tolerance
        self.pid.setSetpoint(self.distance)
        ... # probably need to do more

    def execute(self):
        # determine how far (you might need to make sure that signs are working 
        # the way you think
        ...
        # Need to add some code to ake sure that drives straight
        self.drivetrain.arcadeDrive(output, turn)

    def isFinished(self):
        return # maybe a calculation, maybe asking the PID if within tolerance

    def end(self, interrupted):
        self.drivetrain.arcadeDrive(0, 0)
```
Be sure to compare your final desired distance to what the robot is reporting from the encoders.

#### Turn number of degrees Command:

```python

import commands2
from wpilib.controller import PIDController

class TurnAngle(Command):

    def __init__(self, drivetrain: Drivetrain, angle_to_turn: float = 45):
        super().__init__()
        self.drivetrain = drivetrain
        self.angle = angle_to_turn * math.pi / 180.0
        self.addRequirements(drivetrain)
        self.pidcontroller = PIDController(.8, 0, 0.01)
        self.pidcontroller.setTolerance(1 * math.pi / 180.0)

    def initialize(self):
        self.drivetrain.gyro.reset()

    def execute(self):
        # Use the gyro to get a sense of where you are
        
        # Calculate turn effort using a PID Controller
        turn_effort = # PID Controller
        
        # clamp the speed
        turn_effort = max(min(turn_effort, 0.7), -0.7)
        
        # Make the robot move
        self.drivetrain.arcadeDrive(0, -turn_effort)

    def isFinished(self):
        # Use the PIDController to figure out if you are done.
        ...

    def end(self, interrupted):
        self.drivetrain.arcadeDrive(0, 0)
```

#### Move Until Top of the Ramp Command:

```python
from commands2 import Command

class ClimbRamp(Command):
    def __init__(self, drivetrain):
        super().__init__()
        ## make the drivetrain a part of the class
        # Add requirement for the command

    def initialize(self):
        # Optional: Reset gyro angle if needed
        ...

    def execute(self):
        # Drive forward at fixed speed. Make it fast enough to get up the ramp
        # you might want to use logic similar to drive straight to keep things 
        # moving in the right direction
        ...

    def isFinished(self):
        # Measure the rate of change of pitch (around Y-axis)
        pitch_rate = self.drivetrain.getGyroAngleYRate()  # 
        # There is probably some number and direction that will indicate 
        # you are at the top of the ramp.
        # log and graph to determine what that is.
        return  # test if the angle indicates the Robot is leveling off

    def end(self, interrupted):
        self.drivetrain.arcadeDrive(0.0, 0.0)
```


### Project Presentation:

- Functionality: Demonstrate the robot completing the task.
- Code Explanation: Explain the implementation of PID controllers, use of the commands2 library, and accelerometer integration.
- Collaboration: Discuss the group dynamics and how each member contributed to the project.

### Evaluation Criteria:

- Task Completion: Did the robot successfully complete the task?
- Precision: How accurate were the movements and bump detection?
- Code Quality: Is the code well-documented and structured?
- Understanding: Can the group explain their approach and the role of each component?

