import os

from commands2 import TimedCommandRobot, CommandScheduler, RunCommand
from wpilib import Joystick

from autonomous.fullcommand import ClimbPlatformSequence
from autonomous.turnangle import TurnAngle
from autonomous.drivestraight import DriveStraight
from subsystems.drivetrain import Drivetrain

os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class MyRobot(TimedCommandRobot):

    def robotInit(self):
        '''This method is called as the robot turns on and is often used to setup the
        joysticks and other presets.'''
        self.controller = Joystick(0)
        self.drivetrain = Drivetrain()

        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.arcadeDrive(
                    -self.controller.getRawAxis(1),  # forward/backward
                    self.controller.getRawAxis(0)  # rotation
                ),
                self.drivetrain
            )
        )

    def robotPeriodic(self):
        '''This is called every cycle of the code. In general the code is loop
        through every .02 seconds.'''
        # self.drivetrain.periodic()
        CommandScheduler.getInstance().run()

    def autonomousInit(self):
        '''This is called once when the robot enters autonomous mode.'''
        #self.autoCommand = ClimbPlatformSequence(self.drivetrain)
        #self.autoCommand = DriveStraight(self.drivetrain, 20)
        #self.autoCommand.schedule()
        ...

    def autonomousPeriodic(self):
        '''This is called every cycle while the robot is in autonomous.'''
        pass

    def teleopInit(self):
        '''This is called once at the start of Teleop.'''
        pass

    def teleopPeriodic(self):
        '''This is called once every cycle during Teleop'''
        pass
