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

    private final Command feederStation;
    private final Command homeStation;
    private final Command L3Station;

    // Create the driver controller
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    // Create operator controller
    private final CommandXboxController m_operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);

    private SendableChooser<Command> autoChooser;
    private SendableChooser<double> delayChooser;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(6.);

    private Command visionReefScoreLeft() {
        return ReefScoreCorrectionCommand.left(m_robotDrive, m_limelight,
                () -> 1, () -> -m_driverController.getLeftX());
    }

    private Command visionReefScoreRight() {
        return ReefScoreCorrectionCommand.right(m_robotDrive, m_limelight,
                () -> 1, () -> -m_driverController.getLeftX());
    }

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

        feederStation = CoralManagement.runToPositionCommand(() -> ElevatorPosition.FeederStation)
                .alongWith(Commands.runOnce(() -> CoralManagement.targetPosition = ElevatorPosition.Home));

        homeStation = CoralManagement.runToPositionCommand(() -> ElevatorPosition.Home)
                .alongWith(Commands.runOnce(() -> CoralManagement.targetPosition = ElevatorPosition.Home));

        L3Station = CoralManagement.runToPositionCommand(() -> ElevatorPosition.Level3)
                .alongWith(Commands.runOnce(() -> CoralManagement.targetPosition = ElevatorPosition.Level3));

        configureNamedCommands();

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
        initAutoChooser();

        // Auto delay chooser
        initAutoChooser();
    }

    private void configureNamedCommands() {
        // TODO: This will contain all auto named commands.

        NamedCommands.registerCommand("Vision Align Left", visionReefScoreLeft().withTimeout(1.0));
        NamedCommands.registerCommand("Vision Align Right", visionReefScoreRight().withTimeout(1.0));

        NamedCommands.registerCommand("Move L3", L3Station);
        NamedCommands.registerCommand("Move Feeder Station", feederStation);

        NamedCommands.registerCommand(
                "Score Timed", m_coralInfeed.autoScoreCommand());

        NamedCommands.registerCommand("Smart Intake", m_coralInfeed.smartIntakeCommand());
    }

    private void initAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void initAutoStartDelay() {
        delayChooser = new SendableChooser<>();
        delayChooser.setDefaultOption("0 seconds", 0.0);
        delayChooser.addOption("2 seconds", 2.0);
        delayChooser.addOption("5 seconds", 5.0);
        delayChooser.addOption("10 seconds", 10.0);
        SmartDashboard.putData("Start Delay", delayChooser);
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
        m_driverController.leftTrigger().onTrue(m_algae.intakeCommand(1.0));

        /* Start: Reset Odometry */
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

        /* Y: Vision Align */
        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(
                m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));

        /* X: X-Drive */
        m_driverController.x().toggleOnTrue(m_robotDrive.setXCommand());

        /* B (left): Vision Score Left */
        m_driverController.b().whileTrue(visionReefScoreLeft());

        /* A (right): Vision Score Right */
        m_driverController.a().whileTrue(visionReefScoreRight());

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
        m_operatorController.a().onTrue(feederStation);

        /* B: Go to Home */
        m_operatorController.b().onTrue(homeStation);

        /* LT/RT: Coral Outfeed/Infeed */
        m_operatorController.leftTrigger().whileTrue(m_coralInfeed.scoreCommand())
                .onFalse(m_coralInfeed.stopIntakeCommand());

        m_operatorController.rightTrigger().whileTrue(m_coralInfeed.intakeCommand())
                .onFalse(m_coralInfeed.stopIntakeCommand());

        /* LB: Algae Outfeed */
        m_operatorController.leftBumper().whileTrue(m_algae.intakeCommand(-0.8)).onFalse(m_algae.stopIntakeCommand());

        /* RB: Slow Coral Outfeed */
        m_operatorController.rightBumper().whileTrue(m_coralInfeed.setIntakeCommand(-0.2))
                .onFalse(m_coralInfeed.stopIntakeCommand());

        // ================
        // OPERATOR MANUAL
        // ================

        /* Left Stick Y (Axis 1): Elevator */
        m_operatorController.axisMagnitudeGreaterThan(1, 0.2)
                .whileTrue(m_elevator.runElevatorCommand(() -> -0.5 * m_operatorController.getLeftY()))
                .onFalse(m_elevator.holdPositionCommand());

        /* Right Stick Y (Axis 5): Coral Infeed */
        m_operatorController.axisMagnitudeGreaterThan(5, 0.2)
                .whileTrue(m_coralInfeed.runPivotCommand(() -> -0.35 * m_operatorController.getRightY()))
                .onFalse(m_coralInfeed.holdPositionCommand());
    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command getDelay() {
        return delayChooser.getSelected();
    }

}