package frc.robot;

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
    // private final AlgaeInfeed m_algae = new AlgaeInfeed();

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
        return MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband);
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
        initAutoChooser();

        configureNamedCommands();

        // Configure the buttons & default commands
        configureButtonBindings();

        // Config buttons
    }

    private void configureNamedCommands() {
        // TODO: This will contain all auto named commands.

        NamedCommands.registerCommand("Score L3 Left",
                visionReefScoreLeft
                        .alongWith(m_elevator.runToPositionCommand(ElevatorPosition.Level3))
                        .alongWith(m_coralInfeed.runToPositionCommand(ElevatorPosition.Level3))
                        .andThen(m_coralInfeed.scoreCommand()));

        NamedCommands.registerCommand("Score L3 Right",
                visionReefScoreRight
                        .alongWith(m_elevator.runToPositionCommand(ElevatorPosition.Level3))
                        .alongWith(m_coralInfeed.runToPositionCommand(ElevatorPosition.Level3))
                        .andThen(m_coralInfeed.scoreCommand()));

        NamedCommands.registerCommand("Intake",
                m_elevator.runToPositionCommand(ElevatorPosition.FeederStation)
                        .alongWith(m_coralInfeed.runToPositionCommand(ElevatorPosition.FeederStation))
                        .andThen(m_coralInfeed.autoIntakeCMD()));

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
                                -getRotationSpeed(),
                                true),
                        m_robotDrive));

        // ===================
        // DRIVER CONTROLLER
        // ===================

        /* LT: Intake, LB: Outtake */
        // m_driverController.leftTrigger(0.1).whileTrue(m_coralInfeed.intakeCommand(0.2));
        // m_driverController.leftBumper().whileTrue(m_coralInfeed.intakeCommand(-0.2));

        /* Start: Reset Odometry */
        m_driverController.start().onTrue(Commands.runOnce(() -> m_robotDrive.setOdometry(new Pose2d())));

        // Driver
        // Limelight YAW alignment

        m_driverController.y().toggleOnTrue(m_robotDrive.visionRotateCommand(
                m_limelight, () -> getXSpeed(),
                () -> getYSpeed()));

        m_driverController.x().toggleOnTrue(m_robotDrive.setXCommand());

        m_driverController.leftTrigger(0.1)
                .whileTrue(visionReefScoreLeft);

        m_driverController.rightTrigger(0.1)
                .whileTrue(visionReefScoreRight);
        /*
         * m_driverController.a()
         * .toggleOnTrue(CoralManagement.automaticElevatorCommand(m_limelight.
         * positionResolved()));
         */

        // =====================
        // OPERATOR CONTROLLER
        // =====================

        /* D-Pad Up/Down: Move Elevator */
        // m_operatorController.povUp().onTrue(m_elevator.runElevatorCommand(0.2));
        // m_operatorController.povDown().onTrue(m_elevator.runElevatorCommand(-0.2));

        /* LB/RB: Move Coral Pivot */
        // m_operatorController.rightBumper().whileTrue(m_coralInfeed.runPivotCommand(0.2));
        // m_operatorController.leftBumper().whileTrue(m_coralInfeed.runPivotCommand(-0.2));

        /* LT/RT: Move Algae Pivot */
        // m_operatorController.rightTrigger().whileTrue(m_algae.runPivotCommand(0.2));
        // m_operatorController.leftTrigger().whileTrue(m_algae.runPivotCommand(-0.2));

        /* X/B: Algae Infeed/Outfeed */
        // m_operatorController.leftTrigger(0.1).whileTrue(m_algae.intakeCommand(0.1));
        // m_operatorController.rightTrigger(0.1).whileTrue(m_algae.intakeCommand(-0.1));

        /* Y: Cycle Elevator */
        // m_operatorController.y().onTrue(CoralManagement.cycleAndRunToPositionCommand());
        /* TMP: Run Elevator to L2 */
        // m_operatorController.y().onTrue(m_elevator.runToPositionCommand(ElevatorPosition.Level2));

        /* A: Run to Feeder Station */
        // m_operatorController.a().onTrue(CoralManagement.runToPositionCommand(ElevatorPosition.FeederStation));
        // m_operatorController.a().onTrue(m_elevator.runToPositionCommand(ElevatorPosition.FeederStation));

        /* TMP: D-Pad Left/Right: Run Coral Infeed to L2/Feeder */
        // m_operatorController.povLeft().onTrue(m_coralInfeed.runToPositionCommand(ElevatorPosition.Level2));
        // m_operatorController.povRight().onTrue(m_coralInfeed.runToPositionCommand(ElevatorPosition.FeederStation));

        /* TMP: LS/RS: Deploy/Stow Algae Infeed */
        // m_operatorController.leftStick().onTrue(m_algae.deployCommand());
        // m_operatorController.rightStick().onTrue(m_algae.stowCommand());

    }

    // Get the selected auto command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}