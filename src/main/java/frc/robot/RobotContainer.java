package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final VisionSubsystem m_limelight = new VisionSubsystem();

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);


    private final SendableChooser<Command> autoChooser;


    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);

    private final double baseSpeedPercent = 0.25;

    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent
                        + m_driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }


    
    private double getRotationSpeed() {
        return scaleDriverController(-m_driverController.getRightX(), thetaLimiter);
    }

    private double getYSpeed() {
        return scaleDriverController(-m_driverController.getLeftX(), yLimiter);
    }

    private double getXSpeed() {
        return scaleDriverController(-m_driverController.getLeftY(), xLimiter);
    }



    // https://pathplanner.dev/pplib-triggers.html#pathplannerauto-triggers
    public RobotContainer() {
        // Set the vision subsystem in the drive subsystem
        m_robotDrive.setVisionSubsystem(m_limelight);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Put subsystem woth the method in it
        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

        // Config the buttosn
        configureButtonBindings();

        // Config buttons
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.joystickDrive(
                                MathUtil.applyDeadband(getXSpeed(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(getYSpeed(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(-getRotationSpeed(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    private void configureButtonBindings() {
        // Setting up driver commands

        // Y for angle hold
        m_driverController.y().toggleOnTrue(new RotateToAngle(m_robotDrive, () -> getXSpeed(),
                () -> getYSpeed(), () -> m_limelight.getTX()));
        // Start button to reset odometry
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

    }

    public double get_display_yaw() {
        return m_robotDrive.getHeading();
    }

    // Getter function to see if the target is valid
    public boolean display_targetValid() {
        return m_limelight.isTargetValid();
    }

    // Offset X in degrees
    public double tx() {
        return m_limelight.getTX();
    }

    // Offset Y in degrees
    public double ty() {
        return m_limelight.getTY();
    }

    public Pose2d update_field() {
        return m_robotDrive.getRobotPoseEstimate();
    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
