package frc.robot;

import java.lang.annotation.Target;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ReefScoreCorrectionCommand;
import frc.robot.commands.ReefScoreCorrectionCommand;
import frc.robot.subsystems.AlgaeInfeed;
import frc.robot.subsystems.CoralInfeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.util.CoralManagement;
import frc.robot.util.CoralManagement.ElevatorPosition;
import frc.robot.util.Utils;
import frc.robot.vision.Limelight;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final CoralInfeed m_coralInfeed = new CoralInfeed();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final AlgaeInfeed m_algae = new AlgaeInfeed();
    private ElevatorPosition m_targetPosition = ElevatorPosition.Home;

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    // Create operator controller
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

    private SendableChooser<Command> autoChooser;

    private final Command visionReefScoreLeft = ReefScoreCorrectionCommand.left(m_robotDrive, m_limelight,
            () -> m_driverController.getLeftTriggerAxis(), () -> -m_driverController.getLeftX());
    private final Command visionReefScoreRight = ReefScoreCorrectionCommand.right(m_robotDrive, m_limelight,
            () -> m_driverController.getRightTriggerAxis(), () -> -m_driverController.getLeftX());

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

    private double getRotationSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getRightX(), thetaLimiter,
                m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband);
    }

    private double getYSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getLeftX(), yLimiter,
                m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband);
    }

    private double getXSpeed() {
        return MathUtil.applyDeadband(Utils.scaleDriverController(-m_driverController.getLeftY(), xLimiter,
                m_driverController.getLeftTriggerAxis()), OIConstants.kDriveDeadband);
    }

    // https://pathplanner.dev/pplib-triggers.html#pathplannerauto-triggers
    public RobotContainer() {

        CoralManagement.init(m_coralInfeed, m_elevator);

        initAutoChooser();

        configureNamedCommands();

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
    }

    private void configureNamedCommands() {
        // TODO: This will contain all auto named commands.

        NamedCommands.registerCommand("Reef Left", visionReefScoreLeft);
        NamedCommands.registerCommand("Reef Right", visionReefScoreRight);

        NamedCommands.registerCommand("L1", Commands.runOnce(() -> m_targetPosition = ElevatorPosition.Level1));
        NamedCommands.registerCommand("L2", Commands.runOnce(() -> m_targetPosition = ElevatorPosition.Level2));
        NamedCommands.registerCommand("L3", Commands.runOnce(() -> m_targetPosition = ElevatorPosition.Level3));
        NamedCommands.registerCommand("L4", Commands.runOnce(() -> m_targetPosition = ElevatorPosition.Level4));
        NamedCommands.registerCommand("Feeder Station",
                Commands.runOnce(() -> m_targetPosition = ElevatorPosition.FeederStation));

        NamedCommands.registerCommand("Home", Commands.runOnce(() -> m_targetPosition = ElevatorPosition.Home));

        NamedCommands.registerCommand("Move Elevator", CoralManagement.runToPositionCommand(() -> m_targetPosition));

        NamedCommands.registerCommand(
                "Score", m_coralInfeed.scoreCommand());

        NamedCommands.registerCommand("Intake", m_coralInfeed.intakeCommand(0.8));
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

        // m_algae.setDefaultCommand(m_algae.intakeCommand(0.3));

        // ===================
        // DRIVER CONTROLLER
        // ===================

        /* LT/RT: Move Algae Pivot */
        m_driverController.rightBumper().whileTrue(m_algae.runPivotCommand(0.2)).onFalse(m_algae.stopPivotCommand());
        m_driverController.leftBumper().whileTrue(m_algae.runPivotCommand(-0.2)).onFalse(m_algae.stopPivotCommand());

        /* X/B: Algae Infeed/Outfeed */
        m_driverController.leftTrigger(0.1).whileTrue(m_algae.intakeCommand(0.8)).onFalse(m_algae.intakeCommand(0.6));
        m_driverController.rightTrigger(0.1).whileTrue(m_algae.intakeCommand(-0.4))
                .onFalse(m_algae.stopIntakeCommand());

        /* Start: Reset Odometry */
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

        // Aimbot
        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(
                m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));

        // Set X Command
        m_driverController.x().toggleOnTrue(m_robotDrive.setXCommand());

        // Alignement
        m_driverController.leftStick().whileTrue(visionReefScoreLeft);

        m_driverController.rightStick().whileTrue(visionReefScoreRight);

        // =====================
        // OPERATOR CONTROLLER
        // =====================

        /* D-Pad Up/Down: Move Elevator */
        m_operatorController.povUp().onTrue(m_elevator.runElevatorCommand(0.2))
                .onFalse(m_elevator.stopElevatorCommand());
        m_operatorController.povDown().onTrue(m_elevator.runElevatorCommand(-0.2))
                .onFalse(m_elevator.stopElevatorCommand());

        m_operatorController.leftBumper().whileTrue(m_coralInfeed.runPivotCommand(-0.5))
                .onFalse(m_coralInfeed.stopPivotCommand());
        m_operatorController.rightBumper().whileTrue(m_coralInfeed.runPivotCommand(0.5))
                .onFalse(m_coralInfeed.stopPivotCommand());

        /* Y: Cycle Elevator */
        m_operatorController.y().onTrue(CoralManagement.cycleAndRunToPositionCommand());
        /* TMP: Run Elevator to L2 */
        // m_operatorController.y().onTrue(m_elevator.runToPositionCommand(ElevatorPosition.Level2));

        /* TMP: D-Pad Left/Right: Run Coral Infeed to L2/Feeder */
        m_operatorController.povLeft().onTrue(m_coralInfeed.runToPositionCommand(() -> ElevatorPosition.Level2));
        m_operatorController.povRight()
                .onTrue(m_coralInfeed.runToPositionCommand(() -> ElevatorPosition.FeederStation));

        /* TMP: LS/RS: Deploy/Stow Algae Infeed */
        // m_operatorController.leftStick().onTrue(m_algae.toggleCommand());
        m_operatorController.leftStick().onTrue(m_algae.deployCommand());
        m_operatorController.rightStick().onTrue(m_algae.stowCommand());

        m_operatorController.a().whileTrue(m_coralInfeed.intakeCommand(-0.8));
        m_operatorController.b().whileTrue(m_coralInfeed.intakeCommand(0.8));

        m_operatorController.leftTrigger(0.1).whileTrue(m_algae.intakeCommand(-0.8));
        m_operatorController.rightTrigger(-0.1).whileTrue(m_algae.intakeCommand(0.8));

    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}