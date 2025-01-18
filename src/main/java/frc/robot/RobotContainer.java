package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utils;
import frc.robot.vision.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight m_limelight = new Limelight("limelight");

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    private SendableChooser<Command> autoChooser;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);

    private double getRotationSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getRightX(), thetaLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband);
    }

    private double getYSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getLeftX(), yLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband);
    }

    private double getXSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getLeftY(), xLimiter,
                m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband);
    }

    // https://pathplanner.dev/pplib-triggers.html#pathplannerauto-triggers
    public RobotContainer() {
        initAutoChooser();

        // Put subsystem woth the method in it
        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
    }

    private void initAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
        // Setting up driver commands

        // Default drive command
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.joystickDrive(
                                MathUtil.applyDeadband(getXSpeed(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(getYSpeed(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(-getRotationSpeed(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

        // Y for angle hold
        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));
        // Start button to reset odometry
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}