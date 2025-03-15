package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.ejml.dense.row.SpecializedOps_DDRM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ReefAlignmentConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.Limelight;

public class ReefScoreCorrectionCommand extends Command {

    private double backupVectorX = 0;
    private double backupVectorY = 0;
    private double backupDistance = 0;
    private boolean foundTarget = false;
    private DoubleSupplier m_trimValue;

    private DoubleSupplier m_speed;

    private final DriveSubsystem m_driveSubsystem;
    private final Limelight m_limelight;
    private double m_targetOffset;

    public ReefScoreCorrectionCommand(DriveSubsystem driveSubsystem, Limelight limelight, DoubleSupplier speed,
            double targetOffset, DoubleSupplier trimValue) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_limelight = limelight;
        this.m_speed = speed;
        this.m_targetOffset = targetOffset;
        this.m_trimValue = trimValue;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        backupVectorX = 1;
        backupVectorY = 0;
        // Commented this out so that if command is rescheduled it wont tweak
        // backupDistance = ReefAlignmentConstants.kMaxDist;
        foundTarget = false;
    }
    // Called every time the scheduler runs while the command is scheduled.

    // This is our accurate automatic intake system A.A.I.S
    @Override
    public void execute() {

        double error = m_driveSubsystem.nearest60error();
        double rotationOutput = DriveSubsystem.controller.calculate(error);
        // Normalize
        double rotationCommand = rotationOutput / DriveConstants.kMaxAngularSpeed;

        boolean hasTarget = m_limelight.isTargetValid();
        if (hasTarget) {
            double offset = -m_limelight.getTX();
            double distanceToTarget = m_limelight.getDistanceReef();
            SmartDashboard.putNumber("Distance", distanceToTarget);

            // We need to move where the center of the target is
            // We have the TX and we need to solve the triangle (distance)
            double xComponent = (Math.cos(Math.toRadians(offset)) * distanceToTarget) - 0.42;
            xComponent = MathUtil.clamp(xComponent, 0, 999);
            // IN Ms
            double yComponent = (Math.sin(Math.toRadians(offset)) * distanceToTarget) + m_targetOffset;

            // Normalize the vector
            double magnitude = Math.sqrt(Math.pow(xComponent, 2) + Math.pow(yComponent, 2));
            double strafeSpeedX = xComponent / magnitude;
            double strafeSpeedY = yComponent / magnitude;

            backupVectorX = strafeSpeedX;
            backupVectorY = strafeSpeedY;
            backupDistance = xComponent;
            foundTarget = true;
        }

        SmartDashboard.putNumber("X Comp", backupVectorX);
        SmartDashboard.putNumber("Y Comp", backupVectorY);

        double speedScaling = m_speed.getAsDouble() * 0.4;
        if (foundTarget) {
            double kP = backupDistance / ReefAlignmentConstants.kMaxDist;
            kP = MathUtil.clamp(kP, 0, 1);
            speedScaling = speedScaling * kP;
            // Slow down even more if we lost the target
            speedScaling = hasTarget == false ? speedScaling * 0.5 : speedScaling;
        } else { // Hasnt see a target yet
            speedScaling *= 0.5;
        }
        SmartDashboard.putNumber("Speed scaling", speedScaling);

        // if (backupVectorX > 0.1) {
        m_driveSubsystem.joystickDrive(
                -backupVectorX * speedScaling - 0.03,
                -backupVectorY * speedScaling + (-m_trimValue.getAsDouble() * 0.2),
                rotationCommand,
                false);
        // } else {
        // m_driveSubsystem.joystickDrive(
        // -0.03,
        // -m_trimValue.getAsDouble() * 0.1,
        // rotationCommand,
        // false);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    public static ReefScoreCorrectionCommand left(DriveSubsystem driveSubsystem, Limelight limelight,
            DoubleSupplier speed, DoubleSupplier trimValue) {
        return new ReefScoreCorrectionCommand(driveSubsystem, limelight, speed, 0.17, trimValue);
    }

    public static ReefScoreCorrectionCommand right(DriveSubsystem driveSubsystem, Limelight limelight,
            DoubleSupplier speed, DoubleSupplier trimValue) {
        return new ReefScoreCorrectionCommand(driveSubsystem, limelight, speed, -0.17, trimValue);
    }
}