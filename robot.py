import wpilib
import wpilib.drive
from wpilib import SmartDashboard, Field2d
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d, Rotation2d
## from wpimath.geometry import Rotation2d


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing the use of the DifferentialDrive class.
    Runs the motors with arcade steering.
    """


    def robotInit(self):
        """Robot initialization function"""
        # Allow the display of the field in the simulation
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)


        # Create the Odometry tracker
        self.Robot_Angle = Rotation2d()
        self.robot_pose = Pose2d()
        self.odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(self.Robot_Angle, 0, 0)
        self.simulated_encoder_distance = 0


        leftMotor = wpilib.PWMSparkMax(0)
        rightMotor = wpilib.PWMSparkMax(1)
        self.robotDrive = wpilib.drive.DifferentialDrive(leftMotor, rightMotor)
        self.stick = wpilib.Joystick(0)
        rightMotor.setInverted(True)


    def teleopPeriodic(self):
        self.robotDrive.arcadeDrive(self.stick.getY(), self.stick.getX())


        # Dummy code to make the robot move 
        # self.simulated_encoder_distance = self.simulated_encoder_distance + 0.05
        self.simulated_encoder_distance = self.simulated_encoder_distance + (-self.stick.getY()/20)
        self.robot_pose = self.odometry.update(self.Robot_Angle, self.simulated_encoder_distance, self.simulated_encoder_distance)
        self.field.setRobotPose(self.odometry.getPose())
