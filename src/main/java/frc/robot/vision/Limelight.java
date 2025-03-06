/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.
   Open Source Software - may be modified and shared by FRC teams. The code
   must be accompanied by the FIRST BSD license file in the root directory of
   the project.
   ----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable m_limelightTable;

    /**
     * Creates a new VisionSubsystem.
     */
    public Limelight() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    // Degrees offset on the X axis from the target origin
    public double getTX() {
        return m_limelightTable.getEntry("tx").getDouble(0);
    }

    // Degrees offset on the Y axis from the target origin
    public double getTY() {
        return m_limelightTable.getEntry("ty").getDouble(0);
    }

    // Returns if a target is found (-29.8 to 29.8 degrees in any axis)
    public boolean isTargetValid() {
        return (m_limelightTable.getEntry("tv").getDouble(0) == 1.0);
    }

    // Estimates in order: x, y, z, roll, pitch, yaw
    public Pose2d getBotPose() {
        var botPose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    }

    // Latency (ms)
    public double getLatency() {
        double tl = m_limelightTable.getEntry("tl").getDouble(0); // Pipeline latency in milliseconds
        double cl = m_limelightTable.getEntry("cl").getDouble(0); // Capture latency in milliseconds
        return tl + cl;

    }

    public boolean getSide() {
        // True = left, false = right
        // Negate to correct side
        return -getTX() < 0 ? true : false;
    }

    public double getDistance() {
        // Use robotPose
        // TODO: Implement distance calculation
        return 0;
    }

    public double getTagId() {
        return m_limelightTable.getEntry("tid").getDouble(0);
    }

    public boolean getYawCorrectionStatus() {
        return Math.abs(getTX()) < 1.0;
    }

    public boolean getDistanceCorrectionStatus() {
        return Math.abs(getDistance()) < 1.0;
    }

    public boolean positionResolved() {
        // After the computer thinks it is aligned, it can assist the driver by moving
        // the elevator up automatically
        return getYawCorrectionStatus() && getDistanceCorrectionStatus();
    }

}
