package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class ComputerAlign extends Command {

    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // Filtering vars 
    private double previousTx = 0.0;
    private double previousTy = 0.0;
    private final double alpha = 0.5; // Smoothing factor 0-1 (the time interval per update)

    // d/dx for yaw
    private double previousErrorYaw = 0.0;

    // Slew rate limiting 
    private double previousVelocity_X = 0.0;
    private double previousVelocity_Y = 0.0;
    private double previousVelocity_R = 0.0;
    private final double maxAcceleration = 0.02; // Max change in velocity 

    // Control gains
    private final double kP_Side = 0.03;     
    private final double kP_Distance = 0.05; 
    private final double kP_Yaw = 0.02;       
    private final double kD_Yaw = 0.01;       // d/dx for yaw

    // Deadbands
    private final double deadbandYaw = 1.0;     // Degrees
    private final double deadbandSide = 1.0;    // Degrees
    private final double deadbandDistance = 1.0;

    public ComputerAlign(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
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

    // Filter for y
    private double getFilteredTy() {
        double currentTy = m_visionSubsystem.getTY();
        double filteredTy = alpha * currentTy + (1 - alpha) * previousTy;
        previousTy = filteredTy;
        return filteredTy;
    }

    // Align side 
    private double alignSide() {
        double tx = getFilteredTx();
        double errorSide = tx;

        if (Math.abs(errorSide) < deadbandSide) {
            return 0.0;
        }

        double velocity_Y = -kP_Side * errorSide;

        // Limit maximum speed
        velocity_Y = Math.max(Math.min(velocity_Y, 0.3), -0.3);

        // Apply slew rate limiting
        velocity_Y = limitAcceleration(velocity_Y, previousVelocity_Y);
        previousVelocity_Y = velocity_Y;

        return velocity_Y;
    }

    // Align distance (forward/backward)
    private double alignDistance() {
        double ty = getFilteredTy();
        double errorDistance = ty;

        if (Math.abs(errorDistance) < deadbandDistance) {
            return 0.0;
        }

        double velocity_X = -kP_Distance * errorDistance;

        // Limit maximum speed
        velocity_X = Math.max(Math.min(velocity_X, 0.3), -0.3);

        // Apply slew rate limiting
        velocity_X = limitAcceleration(velocity_X, previousVelocity_X);
        previousVelocity_X = velocity_X;

        return velocity_X;
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
            // Get the correction velocities 
            double velocity_Y = alignSide();
            double velocity_X = alignDistance();
            double velocity_R = alignYaw(m_drivetrainSubsystem.getHeading());

            
            // Drive
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
