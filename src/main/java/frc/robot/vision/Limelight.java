// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Limelight extends VisionSystem {
    int targetPipeline = 0;

    // private final double TELEOP_MT2_ROTATION_THRESHOLD = 0.5;
    // private final double AUTON_MT2_ROTATION_THRESHOLD = 0.0;

    // private double angularVelocityThreshold = 0.5; // how many rad/sec to update
    // MT2 pose

    public Limelight(String cameraName, Transform3d robotToCamera) {
        super(cameraName, robotToCamera);
    }

    public Limelight(String cameraName) {
        super(cameraName, new Transform3d());
    }

    public boolean getHasTarget() {
        return LimelightHelpers.getTV(cameraName);
    }

    public Optional<Rotation2d> getTargetX() {
        // Limelight's TX is inverted compared to the robot;
        // we want everything in terms of the robot coordinate system.
        if (LimelightHelpers.getTV(cameraName))
            return Optional.of(Rotation2d.fromDegrees(-LimelightHelpers.getLimelightNTDouble(cameraName, "txnc")));

        // we use txnc since it's independent of the crosshair
        return Optional.empty();
    }

    public Optional<Rotation2d> getTagYaw(int tagID) {
        return getTargetX();
    }

    public Optional<Double> getTagDistance(int tagID) {
        var dist = super.getTagDistance(tagID);
        var tx = getTargetX();
        if (dist.isEmpty() || tx.isEmpty())
            return dist;

        return Optional.of(dist.get() / Math.cos(-tx.get().getRadians()));
    }

    public Optional<Rotation2d> getTargetY() {
        if (LimelightHelpers.getTV(cameraName))
            return Optional.of(Rotation2d.fromDegrees(LimelightHelpers.getLimelightNTDouble(cameraName, "tync")));

        return Optional.empty();
    }

    public Optional<Rotation2d> getTagPitch(int tagID) {
        return getTargetY();
    }

    // public void setRobotRotationMT2(double degrees) {
    // LimelightHelpers.SetRobotOrientation(cameraName, degrees, 0, 0, 0, 0, 0);
    // }

    // public LimelightHelpers.PoseEstimate getBotposeEstimateMT2() {
    // return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    // }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(cameraName, pipeline);
        targetPipeline = pipeline;
    }

    public BooleanSupplier getPipelineReady() {
        return () -> targetPipeline == getPipeline();
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
    }

    // public void setTeleopMT2Threshold() {
    // angularVelocityThreshold = TELEOP_MT2_ROTATION_THRESHOLD;
    // }

    // public void setAutonMT2Threshold() {
    // angularVelocityThreshold = AUTON_MT2_ROTATION_THRESHOLD;
    // }
}