import commands2
import math
from wpimath.controller import PIDController

class TurnAngle(Command):
    def __init__(self, drivetrain: Drivetrain, angle_to_turn: float = 45):
        super().__init__()
        self.drivetrain = drivetrain
        self.angle = angle_to_turn * math.pi / 180.0
        self.addRequirements(drivetrain)
        self.pidcontroller = PIDController(.8, 0, 0.01)
        self.pidcontroller.setTolerance(1 * math.pi / 180.0)
        self.pidcontroller.setSetpoint(self.angle)
    def initialize(self):
        self.drivetrain.gyro.reset()
    def execute(self):
        # Use the gyro to get a sense of where you are
        current_angle = math.radians(self.drivetrain.gyro.getAngle())
        # Calculate turn effort using a PID Controller
        turn_effort = self.pidcontroller.update(current_angle)

        # clamp the speed
        turn_effort = max(min(turn_effort, 0.7), -0.7)

        # Make the robot move
        self.drivetrain.arcadeDrive(0, -turn_effort)

    def isFinished(self):
        # Use the PIDController to figure out if you are done.
        current_angle = math.radians(self.drivetrain.gyro.getAngle())
        return self.pidcontroller.update(current_angle)


    def end(self, interrupted):
        self.drivetrain.arcadeDrive(0, 0)