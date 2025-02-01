// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Mass;

import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final Pigeon2 m_gyro;

    // FOR DRIVE METHOD
    // Slew rate filter variables for controlling lateral acceleration in the drive
    // method
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;
    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Standard deviations of model states and vision measurements (default)
    private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);
    private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

    // Reference to the VisionSubsystem
    private VisionSubsystem m_visionSubsystem;

    private PIDController yawController;


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        double kP = 0.04;
        double kD = 0.0034;

        yawController = new PIDController(kP, 0, kD);
        yawController.enableContinuousInput(-180, 180);
        yawController.setTolerance(2);

        m_gyro = new Pigeon2(15); // Pigeon is on CAN Bus with device ID 15
        m_gyro.setYaw(0);

        // Initialize the pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;// = new RobotConfig(74, 6.8, new ModuleConfig(null, null, m_currentRotation,
                           // null, null, 0), new Translation2d(0, 0));
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getRobotPoseEstimate, // Robot pose supplier
                    this::setOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> setRobotVelocitiesDriver(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond, false),
                    // Method that will drive the robot given ROBOT
                    // RELATIVE ChassisSpeeds. Also optionally outputs
                    // individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for
                                                    // holonomic drive trains
                            new PIDConstants(2., 0.0, 0.0), // Translation PID constants
                            new PIDConstants(2, 0.0, 2) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

    }

    public void setVisionSubsystem(VisionSubsystem visionSubsystem) {
        m_visionSubsystem = visionSubsystem;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("X Speed (real)", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Y Speed (real)", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Rot Speed (real)", getChassisSpeeds().omegaRadiansPerSecond);
        SmartDashboard.putData("Heading PID", yawController);
        // Update the pose estimator with sensor data
        m_poseEstimator.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        // Use vision measurement if available
        if (m_visionSubsystem != null && m_visionSubsystem.isTargetValid()) {
            double[] botPoseArray = m_visionSubsystem.getBotPose(); // TODO change here too
            if (botPoseArray.length == 6) {
                double x = botPoseArray[0]; // Meters
                double y = botPoseArray[1];
                double yaw = botPoseArray[5]; // Degrees
                Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

                // Adjust the timestamp for latency
                double latency = m_visionSubsystem.getLatency();
                double visionCaptureTime = Timer.getFPGATimestamp() - (latency / 1000.0);

                // Add the vision measurement to the pose estimator
                m_poseEstimator.addVisionMeasurement(visionPose, visionCaptureTime);
            }
        }
        // yawController.calculate(m_currentRotation);
    }

    Rotation2d getRotation2D() {
        return getRobotPoseEstimate().getRotation();
    }

    void encoderReset() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();

    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void setOdometry(Pose2d pose) {

        // encoderReset();

        m_poseEstimator.resetPosition(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);

    }

    /**
     * Method to drive the robot using% joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    private void setRobotVelocitiesDriver(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SmartDashboard.putNumber("X Speed (passed)", xSpeed);
        SmartDashboard.putNumber("Y Speed (passed)", ySpeed);
        SmartDashboard.putNumber("Rot Speed (passed)", rot);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                getRobotPoseEstimate().getRotation())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        setModuleStates(swerveModuleStates);
    }

    public void joystickDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        setRobotVelocitiesDriver(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
    }

    public void setYawControllerSetpoint(double setPoint) {
        yawController.setSetpoint(setPoint);
    }

    public void setPointRelative(double offset) {
        yawController.setSetpoint(yawController.getSetpoint() + offset);
    }

    public void drivePIDHeading(double xSpeed, double ySpeed, boolean fieldRelative) {
         double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
         double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
         double output = yawController.calculate(getHeading());
         output = MathUtil.clamp(output, DriveConstants.kYawHeadingMinRate, DriveConstants.kYawHeadingMaxRate);
         setRobotVelocitiesDriver(xSpeedDelivered, ySpeedDelivered, -output * DriveConstants.kMaxAngularSpeed, fieldRelative);
    }





    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    public double getHeading() {
        // Get the heading and normalize it between 0 and 360 to ensure proper readings
        return getRobotPoseEstimate().getRotation().getDegrees();
    }

    public Pose2d getRobotPoseEstimate() {
        // Returns the estimated robot position (x,y,yaw)
        // WARNING the negating of x & y could cause issues with auto driving
        return new Pose2d(m_poseEstimator.getEstimatedPosition().getX(),
                m_poseEstimator.getEstimatedPosition().getY(),
                m_poseEstimator.getEstimatedPosition().getRotation());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

}
