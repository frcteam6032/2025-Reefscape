package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OIConstants;

public class VisionAssistance extends Command {

    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Filtering vars 
    private double previousTx = 0.0;
    private final double alpha = 0.5; // Smoothing factor 0-1 (the time interval per update)

    // d/dx for yaw
    private double previousErrorYaw = 0.0;

    // Slew rate limiting 
    private double previousVelocity_R = 0.0;
    private final double maxAcceleration = 0.02; // Max change in velocity 

    private final double kP_Yaw = 0.02;       
    private final double kD_Yaw = 0.01;       // d/dx for yaw

    // Deadbands
    private final double deadbandYaw = 1.0;     // Degrees

    public VisionAssistance(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;

        addRequirements(drivetrainSubsystem, visionSubsystem);
    }

    // Filter for x
    private double getFilteredTx() {
        double currentTx = m_visionSubsystem.getTX();
        double filteredTx = alpha * currentTx + (1 - alpha) * previousTx;
        previousTx = filteredTx;
        return filteredTx;
    }


    // Align yaw 
    private double alignYaw(double currentYaw) {
        double tx = getFilteredTx();
        double errorYaw = -tx;

        if (Math.abs(errorYaw) < deadbandYaw) {
            previousErrorYaw = errorYaw; // Update for d/dx
            previousVelocity_R = 0.0;
            return 0.0;
        }

        // Calculate derivative (change in error)
        double derivativeYaw = errorYaw - previousErrorYaw;
        previousErrorYaw = errorYaw;

        double velocity_R = kP_Yaw * errorYaw + kD_Yaw * derivativeYaw;

        // Limit maximum rotation speed
        velocity_R = Math.max(Math.min(velocity_R, 0.3), -0.3);

        // Apply slew rate limiting
        velocity_R = limitAcceleration(velocity_R, previousVelocity_R);
        previousVelocity_R = velocity_R;

        return velocity_R;
    }

    // Limit acceleration
    private double limitAcceleration(double desiredVelocity, double previousVelocity) {
        double deltaVelocity = desiredVelocity - previousVelocity;
        deltaVelocity = Math.max(Math.min(deltaVelocity, maxAcceleration), -maxAcceleration);
        return previousVelocity + deltaVelocity;
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.isTargetValid()) {
            double velocity_R = alignYaw(m_drivetrainSubsystem.getHeading());
            // Allow the driver to make movements while the computer attempts to automatically aim at a target
           m_drivetrainSubsystem.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), velocity_R, false, false);
        } else {
            // Abort the command if the target is not found
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
