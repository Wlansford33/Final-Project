import math

import ntcore
import romi
import wpilib
import wpilib.drive
from commands2 import Subsystem
from wpimath.controller import SimpleMotorFeedforwardMeters, PIDController
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics


class Drivetrain(Subsystem):
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591
    MAX_LINEAR_SPEED = 4  # meters per second (down from theoretical max of .89)
    MAX_ANGULAR_SPEED = 40  # radians per second (down from theoretical max of 12.6)

    def __init__(self) -> None:
        super().__init__()

        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.rightMotor = wpilib.Spark(1)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.resetEncoders()
        # Set up Network tables
        self.nt_drivetrain = ntcore.NetworkTableInstance.getDefault().getTable("Drivetrain")
        # Calculate Values
        self.feedforward_left = SimpleMotorFeedforwardMeters(kS=0, kV=1, kA=0.0)  # Your constants
        self.feedforward_right = SimpleMotorFeedforwardMeters(kS=0, kV=1, kA=0)  # Your constants
        # PID Controller to give some feedback control
        self.pid_left = PIDController(0.0, 0.0, 0.0)
        self.pid_right = PIDController(0.0, 0.0, 0.0)
        # Kinematics helps us know what velocity the wheels need to move
        self.kinematics = DifferentialDriveKinematics(trackWidth=0.14)  # ROMI width in meters
        # Need set a value for the speeds here so that we have something to report in periodic
        self.left_speed = 0
        self.right_speed = 0

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
        # chassis_speeds = ChassisSpeeds(4, 0.0, 0.0)

        # 3. Convert chassis speeds to wheel speeds
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis_speeds)

        # 3.5 PID
        actual_left_speed = self.leftEncoder.getRate() * self.kWheelDiameterInch * .0254 * math.pi  # in rotations per second
        actual_right_speed = self.rightEncoder.getRate() * self.kWheelDiameterInch * .0254 * math.pi

        self.left_speed = wheel_speeds.left / (self.kWheelDiameterInch * .0254 * math.pi)  # Rotations per second
        self.right_speed = wheel_speeds.right / (self.kWheelDiameterInch * .0254 * math.pi)

        # 3.5 Add compute correction from a PIDController
        left_pid = self.pid_left.calculate(actual_left_speed, self.left_speed)
        right_pid = self.pid_right.calculate(actual_right_speed, self.right_speed)

        # 4. Use feedforward to get voltages
        left_voltage = self.feedforward_left.calculate(self.left_speed)
        right_voltage = self.feedforward_right.calculate(self.right_speed)

        # print(f"left voltage {left_voltage} right voltage {right_voltage}")
        # print(f"left speed {self.left_speed} right speed {self.right_speed}")
        # print(f"left actual {actual_left_speed} right error {actual_right_speed}")

        # 5. Send voltages to motors
        self.leftMotor.setVoltage((left_voltage + left_pid))
        self.rightMotor.setVoltage(-(right_voltage + right_pid))

    def arcadeDriveWPILIB(self, fwd: float, rot: float) -> None:
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(rot, fwd)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self) -> int:
        return self.leftEncoder.get()

    def getRightEncoderCount(self) -> int:
        return self.rightEncoder.get()

    def getLeftDistanceInch(self) -> float:
        return self.leftEncoder.getDistance()

    def getRightDistanceInch(self) -> float:
        return self.rightEncoder.getDistance()

    def getAverageDistanceInch(self) -> float:
        """Gets the average distance of the TWO encoders."""
        return (self.getLeftDistanceInch() + self.getRightDistanceInch()) / 2.0

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.
        This is the angle you would want for the turn in 2D space
        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()

    def getGyroAngleXRate(self):
        return self.gyro.getRateX()

    def getGyroAngleYRate(self):
        return self.gyro.getRateY()

    def getGyroAngleZRate(self):
        return self.gyro.getRateZ()

    def resetGyro(self) -> None:
        """Reset the gyro"""
        self.gyro.reset()

    def periodic(self) -> None:
        self.nt_drivetrain.putNumber("Left Encoder", self.getLeftEncoderCount())
        self.nt_drivetrain.putNumber("Right Encoder", self.getRightEncoderCount())
        self.nt_drivetrain.putNumber("Left Desired Speed", self.left_speed)
        self.nt_drivetrain.putNumber("Left Actual Speed", self.leftEncoder.getRate())
        self.nt_drivetrain.putNumber("Right Desired Speed", self.right_speed)
        self.nt_drivetrain.putNumber("Right Actual Speed", self.rightEncoder.getRate())
        self.nt_drivetrain.putNumber("Left Distance", self.getLeftDistanceInch())
        self.nt_drivetrain.putNumber("Right Distance", self.getRightDistanceInch())
        self.nt_drivetrain.putNumber("Average Distance", self.getAverageDistanceInch())
        self.nt_drivetrain.putNumber("Gyro Reading X", self.getGyroAngleX())
        self.nt_drivetrain.putNumber("Gyro Reading Y", self.getGyroAngleY())
        self.nt_drivetrain.putNumber("Gyro Reading Z", self.getGyroAngleZ())
        self.nt_drivetrain.putNumber("Gyro Rate X", self.getGyroAngleXRate())
        self.nt_drivetrain.putNumber("Gyro Rate Y", self.getGyroAngleYRate())
        self.nt_drivetrain.putNumber("Gyro Rate Z", self.getGyroAngleZRate())