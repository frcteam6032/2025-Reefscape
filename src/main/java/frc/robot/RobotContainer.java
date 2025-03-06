package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralInfeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeInfeed;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.util.CoralManagement;
import frc.robot.util.Utils;
import frc.robot.vision.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final CoralInfeed m_coralInfeed = new CoralInfeed();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final AlgaeInfeed m_algae = new AlgaeInfeed();

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    // Create operator controller
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

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

        CoralManagement.init(m_coralInfeed, m_elevator);

        // Put subsystem woth the method in it
        // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());

        configureNamedCommands();

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
    }

    private void configureNamedCommands() {
        // NamedCommands.registerCommand("L1 Pivot",
        // m_coralInfeed.runToPositionCommand(ElevatorPosition.Level1));
        NamedCommands.registerCommand("Score", m_coralInfeed.intakeCommand(-0.5));
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
                                getXSpeed(),
                                getYSpeed(),
                                getRotationSpeed(),
                                true),
                        m_robotDrive));

        // Driver
        // Limelight YAW alignment
        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));

        m_driverController.rightStick().whileTrue(m_robotDrive.rotateToAngleCommand(() -> getXSpeed(),
                () -> getYSpeed(),
                () -> new Rotation2d(Math.atan2(m_driverController.getRightX(), m_driverController.getRightY()))));

        m_driverController.leftTrigger(0.1).whileTrue(m_coralInfeed.intakeCommand(0.5));
        m_driverController.leftBumper().whileTrue(m_coralInfeed.intakeCommand(-0.5));
        m_driverController.x().toggleOnTrue(m_robotDrive.setXCommand());
        m_driverController.rightBumper()
                .whileTrue(m_robotDrive.sideAlignmentCommand(Math.abs(m_limelight.getTX()), m_limelight.getSide()));

        // Operator
        m_operatorController.povUp().onTrue(m_elevator.runElevatorCommand(0.2));
        m_operatorController.povDown().onTrue(m_elevator.runElevatorCommand(-0.2));
        // Coral
        m_operatorController.leftBumper().whileTrue(m_coralInfeed.runPivotCommand(0.5));
        m_operatorController.rightBumper().whileTrue(m_coralInfeed.runPivotCommand(-0.5));
        m_operatorController.y().onTrue(CoralManagement.cycleAndRunToPositionCommand());

        // Algae
        m_operatorController.x().whileTrue(m_algae.runPivotCommand(0.5));
        m_operatorController.b().whileTrue(m_algae.runPivotCommand(-0.5));
        m_operatorController.leftTrigger(0.1).whileTrue(m_algae.intakeCommand(0.1));
        m_operatorController.rightTrigger(0.1).whileTrue(m_algae.intakeCommand(-0.1));

        // Start button to reset odometry
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}