package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
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
            () -> 1, () -> -m_driverController.getLeftX());
    private final Command visionReefScoreRight = ReefScoreCorrectionCommand.right(m_robotDrive, m_limelight,
            () -> 1, () -> -m_driverController.getLeftX());

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

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

    /**
     * DONT CHANGE THESE
     */
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

        // ==================
        // DRIVER CONTROLLER
        // ==================

        /* RB: Toggle Algae Infeed */
        m_driverController.rightBumper().onTrue(m_algae.toggleCommand());

        /* LT: Algae Infeed */
        m_driverController.leftTrigger().onTrue(m_algae.intakeCommand(0.8));

        /* Start: Reset Odometry */
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

        /* Y: Vision Align */
        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(
                m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));

        /* X: X-Drive */
        m_driverController.x().toggleOnTrue(m_robotDrive.setXCommand());

        /* B (left): Vision Score Left */
        m_driverController.b().whileTrue(visionReefScoreLeft);

        /* A (right): Vision Score Right */
        m_driverController.a().whileTrue(visionReefScoreRight);

        // ==============
        // DRIVER MANUAL
        // ==============

        /* D-Pad Up/Down: Algae Infeed */
        m_driverController.povUp().whileTrue(m_algae.runPivotCommand(0.3)).onFalse(m_algae.holdPositionCommand());
        m_driverController.povDown().whileTrue(m_algae.runPivotCommand(-0.3)).onFalse(m_algae.holdPositionCommand());

        // ====================
        // OPERATOR CONTROLLER
        // ====================

        /* Y: Cycle Elevator */
        m_operatorController.y().onTrue(CoralManagement.cycleAndRunToPositionCommand());

        /* A: Feeder Station */
        m_operatorController.a().onTrue(CoralManagement.runToPositionCommand(() -> ElevatorPosition.FeederStation)
                .alongWith(Commands.runOnce(() -> CoralManagement.targetPosition = ElevatorPosition.Home)));

        /* B: Go to Home */
        m_operatorController.b().onTrue(CoralManagement.runToPositionCommand(() -> ElevatorPosition.Home)
                .alongWith(Commands.runOnce(() -> CoralManagement.targetPosition = ElevatorPosition.Home)));

        /* LT/RT: Coral Outfeed/Infeed */
        m_operatorController.leftTrigger().whileTrue(m_coralInfeed.intakeCommand(-0.8))
                .onFalse(m_coralInfeed.stopIntakeCommand());

        m_operatorController.rightTrigger().whileTrue(m_coralInfeed.intakeCommand(0.5))
                .onFalse(m_coralInfeed.stopIntakeCommand());

        /* LB/RB: Algae Outfeed/Infeed */
        m_operatorController.leftBumper().whileTrue(m_algae.intakeCommand(-0.8)).onFalse(m_algae.stopIntakeCommand());
        m_operatorController.rightBumper().whileTrue(m_algae.intakeCommand(0.8)).onFalse(m_algae.stopIntakeCommand());

        // ================
        // OPERATOR MANUAL
        // ================

        /* Left Stick Y (Axis 1): Elevator */
        m_operatorController.axisMagnitudeGreaterThan(1, 0.2)
                .whileTrue(m_elevator.runElevatorCommand(() -> -0.5 * m_operatorController.getLeftY()))
                .onFalse(m_elevator.holdPositionCommand());

        /* Right Stick Y (Axis 5): Coral Infeed */
        m_operatorController.axisMagnitudeGreaterThan(5, 0.2)
                .whileTrue(m_coralInfeed.runPivotCommand(() -> -0.5 * m_operatorController.getRightY()))
                .onFalse(m_coralInfeed.holdPositionCommand());
    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}