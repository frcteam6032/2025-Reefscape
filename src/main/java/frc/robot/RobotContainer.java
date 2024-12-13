package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ComputerAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Create the robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final VisionSubsystem m_limelight = new VisionSubsystem();

    // Create the driver controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Create the alignment command
    private final Command ComputerAligner = new ComputerAlign(m_robotDrive, m_limelight);

    public RobotContainer() {
        // Set the vision subsystem in the drive subsystem
        m_robotDrive.setVisionSubsystem(m_limelight);

        // Config the buttosn
        configureButtonBindings();

        // Config buttons
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true, false),
                        m_robotDrive));
    }

    private void configureButtonBindings() {
        // Setting up driver commands
       // new Trigger(m_driverController::getAButton).onTrue(Commands.runOnce(() -> m_robotDrive.setHeading(0.0)));
        new Trigger(m_driverController::getYButton).whileTrue(ComputerAligner);
        new Trigger(m_driverController::getBButton)
                .onTrue(Commands.runOnce(() -> m_robotDrive.resetOdometry(new Pose2d())));
       // new Trigger(m_driverController::getXButton).onTrue(Commands.runOnce(() -> m_robotDrive.incrementHeading(90)));
    }

    // Method to set the YAW (so we can account for offset starting position for the
    // robot relative to the driver/operator)
    public void headingSet(double degree) {
       // m_robotDrive.setHeading(degree);
    }

    public double get_display_yaw() {
        return m_robotDrive.getHeading();
    }

    // Getter function to see if the target is valid
    public boolean targetValid() {
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
        return null;
    }


}
