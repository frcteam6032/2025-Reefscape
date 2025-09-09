// frc/robot/commands/AutoDriveToNearestTag.java
package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.ObstacleAvoidance;
import frc.robot.vision.Limelight;

public class AutoDriveToNearestTag extends Command {
    private final DriveSubsystem drive;
    private final Limelight limelight;
    private final Supplier<ObstacleAvoidance.Readings> sonarSupplier;

    private final HolonomicDriveController hdc;
    private Pose2d targetPoseField = null;
    private boolean acquired = false;

    private final double kSearchOmega = Math.toRadians(45); 
    private final double kStandoffMeters = 0.55;            
    private final double kMaxVel = 2.5;                     
    private final double kMaxOmega = Math.toRadians(360);   
    private final double kCloseEnough = 0.08;               

    public AutoDriveToNearestTag(
        DriveSubsystem drive,
        Limelight limelight,
        Supplier<ObstacleAvoidance.Readings> sonarSupplier
    ) {
        this.drive = drive;
        this.limelight = limelight;
        this.sonarSupplier = sonarSupplier;

        PIDController x = new PIDController(3.5, 0, 0.1);
        PIDController y = new PIDController(3.5, 0, 0.1);
        ProfiledPIDController theta = new ProfiledPIDController(4.0, 0, 0.2, null);
        theta.enableContinuousInput(-Math.PI, Math.PI);

        hdc = new HolonomicDriveController(x, y, theta);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        acquired = false;
        targetPoseField = null;
    }

    @Override
    public void execute() {
        Pose2d robot = drive.getRobotPoseEstimate();

        if (!acquired) {
            if (!limelight.isTargetValid()) {
                drive.drive(0, 0, kSearchOmega, true);
                return;
            }
            
            double tid = limelight.getTagId();
            if (tid <= 0) {
                drive.drive(0, 0, kSearchOmega, true);
                return;
            }

            Pose2d llRobot = limelight.getBotPose();

            Rotation2d goalHeading = Rotation2d.fromDegrees(0); // will be overridden below

        
            double standoff = kStandoffMeters;
            Translation2d forward = new Translation2d(standoff, robot.getRotation());
            targetPoseField = new Pose2d(robot.getTranslation().plus(forward), robot.getRotation());
            goalHeading = robot.getRotation();

            acquired = true;
        }

        if (targetPoseField != null) {
            var speeds = hdc.calculate(drive.getRobotPoseEstimate(), targetPoseField, 0.0, targetPoseField.getRotation());
            double vx = MathUtil.clamp(speeds.vxMetersPerSecond, -kMaxVel, kMaxVel);
            double vy = MathUtil.clamp(speeds.vyMetersPerSecond, -kMaxVel, kMaxVel);
            double omega = MathUtil.clamp(speeds.omegaRadiansPerSecond, -kMaxOmega, kMaxOmega);

            // Obstacle avoidance (field-relative)
            ObstacleAvoidance.Readings r = sonarSupplier.get();
            Translation2d repel = ObstacleAvoidance.repel(r);
            vx += repel.getX();

            vy += repel.getY();
            double err = drive.getRobotPoseEstimate().getTranslation().getDistance(targetPoseField.getTranslation());
            if (err > kCloseEnough) {
                drive.drive(vx, vy, omega, true);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Run while the button is held; let the button binding handle interrupt.
        return false;
    }
}
