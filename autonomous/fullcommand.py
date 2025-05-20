from commands2 import SequentialCommandGroup
from autonomous.drivestraight import DriveStraight
from autonomous.turnangle import TurnAngle
from autonomous.driveupramp import DriveUpRamp
class ClimbPlatformSequence(SequentialCommandGroup):
    def __init__(self, drivetrain):
        super().__init__(
            )