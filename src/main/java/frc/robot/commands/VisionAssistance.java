package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionAssistance extends Command {

    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


    // Filtering
    private double previousTx = 0.0;
    private final double alpha = 0.5;     // Smoothing

    // Slew rate limiting
    private double previousVelocity_R = 0.0;
    private final double maxAcceleration = 0.02;  // Max change 

    // Use the "custom" PID from WPI
    private final PIDController yaw_pid_controller;

    // PID constants 
    private final double kP_Yaw = 0.02;
    private final double kI_Yaw = 0.0;
    private final double kD_Yaw = 0.01;

    // Max the computer can look away from target origin 
    private final double inaccuracyThreshold = 1.0;

    public VisionAssistance(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;
        addRequirements(drivetrainSubsystem, visionSubsystem);

        // Config the controller
        yaw_pid_controller = new PIDController(kP_Yaw, kI_Yaw, kD_Yaw);
        
        // Say our error should be 0
        yaw_pid_controller.setSetpoint(0.0);

        // Max degrees away from setpoint
        yaw_pid_controller.setTolerance(inaccuracyThreshold);

    }


    private double getFilteredTx() {
        double currentTx = m_visionSubsystem.getTX();
        double filteredTx = alpha * currentTx + (1 - alpha) * previousTx;
        previousTx = filteredTx;
        // SmartDashboard.putNumber("Filtered Tx", filteredTx);
        // SmartDashboard.putNumber("Raw Tx", currentTx);
        return filteredTx;
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.isTargetValid() && yaw_pid_controller.atSetpoint() == false) {
            // Get the filtered measurement
            double tx = getFilteredTx();

            // Find computed speed 
            double velocity_R = yaw_pid_controller.calculate(tx);

            // Normalize values between -0.3 and 0.3
            velocity_R = MathUtil.clamp(velocity_R, -0.3, 0.3);

            // Acceleration limiting
            // TODO change to slew rate limiter
            velocity_R = limitAcceleration(velocity_R, previousVelocity_R);
            previousVelocity_R = velocity_R;

            double xMove = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
            double yMove  = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
            // Only modify rotational velocity using the computer
            m_drivetrainSubsystem.drive(xMove, yMove, velocity_R, false, false);
        } else {
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

 
    private double limitAcceleration(double desiredVelocity, double previousVelocity) {
        double deltaVelocity = desiredVelocity - previousVelocity;
        deltaVelocity = MathUtil.clamp(deltaVelocity, -maxAcceleration, maxAcceleration);
        return previousVelocity + deltaVelocity;
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
