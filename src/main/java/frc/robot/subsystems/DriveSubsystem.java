package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Yaw_PID_Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.SwerveUtils;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {

  // Set up the spark maxes 
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

  private final Pigeon2 m_gyro;

 // Init odometry 
  private final SwerveDrivePoseEstimator m_poseEstimator;

  // Rate limiting 
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private final SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;


  private VisionSubsystem m_visionSubsystem;

  // YAW overridd flag 
  private boolean IsVisionEnabled = false;

  private final PIDController Yaw_PID_Controller;

  public DriveSubsystem() {
    m_gyro = new Pigeon2(15);
    m_gyro.setYaw(0);

    // Create the pose estimator
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
        VecBuilder.fill(0.05, 0.05, 0.01),  // state std dev
        VecBuilder.fill(0.5, 0.5, 0.5)      // vision meas std dev
    );

    Yaw_PID_Controller = new PIDController(Yaw_PID_Constants.kP_Yaw, Yaw_PID_Constants.kI_Yaw, Yaw_PID_Constants.kD_Yaw);

  }

  public void setVisionSubsystem(VisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
  }

  @Override
  public void periodic() {
    // Update odometry
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
    );


    // Update estimate with vision data
    if (m_visionSubsystem != null && m_visionSubsystem.isTargetValid()) {
        Pose2d visionPose = m_visionSubsystem.getBotPose();

        double latency = m_visionSubsystem.getLatency();
        double timestamp = Timer.getFPGATimestamp() - (latency / 1000.0);

        m_poseEstimator.addVisionMeasurement(visionPose, timestamp);
      
    }
  }

  public void enableVisionAlign(boolean enable) {
    IsVisionEnabled = enable;
  }


  /**
   * @param xSpeed   forward/back
   * @param ySpeed   left/right
   * @param rot      base rotation (no vision assistance)
   * @param fieldRelative do field oriented drive true/false
   * @param rateLimit     ramp-up enabled/disabled 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCmd, ySpeedCmd;
    if (rateLimit) {
      double inputDir = Math.atan2(ySpeed, xSpeed);
      double inputMag = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

      double directionSlewRate =
          (m_currentTranslationMag != 0.0)
              ? Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag)
              : 500.0;

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputDir, m_currentTranslationDir);

      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
            m_currentTranslationDir, inputDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) {
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
            m_currentTranslationDir, inputDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCmd = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCmd = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      // No rate limit
      xSpeedCmd = xSpeed;
      ySpeedCmd = ySpeed;
      m_currentRotation = rot;
    }

    // Make x/y into velocity
    double vx = xSpeedCmd * DriveConstants.kMaxSpeedMetersPerSecond;
    double vy = ySpeedCmd * DriveConstants.kMaxSpeedMetersPerSecond;

    // Set rotation
    double rotationCommand;
    if (IsVisionEnabled && m_visionSubsystem != null && m_visionSubsystem.isTargetValid()) {
      // If vision is enabled we attempt to align with the target/make tx approach 0
      double tx = m_visionSubsystem.getTX();
      double pidOut = Yaw_PID_Controller.calculate(tx, 0.0); 
      pidOut = MathUtil.clamp(pidOut, -0.3, 0.3); // limit turn speed
      rotationCommand = pidOut * DriveConstants.kMaxAngularSpeed;

    } else {
      // Normal joystick rotation
      rotationCommand = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotationCommand, m_gyro.getRotation2d())
            : new ChassisSpeeds(vx, vy, rotationCommand));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  public double getHeading() {
    return getRobotPoseEstimate().getRotation().getDegrees();
  }

  public Pose2d getRobotPoseEstimate() {
    // Negate x/t to fix robot front/back
    return new Pose2d(
        -m_poseEstimator.getEstimatedPosition().getX(),
        -m_poseEstimator.getEstimatedPosition().getY(),
         m_poseEstimator.getEstimatedPosition().getRotation()
    );
  }

  public void setOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose
    );
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
}
