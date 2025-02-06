/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.
   Open Source Software - may be modified and shared by FRC teams. The code
   must be accompanied by the FIRST BSD license file in the root directory of
   the project.
   ----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public double[] getBotPose() { // TODO change to return a Pose2d
        return m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    // Latency (ms)
    public double getLatency() {
        double tl = m_limelightTable.getEntry("tl").getDouble(0); // Pipeline latency in milliseconds
        double cl = m_limelightTable.getEntry("cl").getDouble(0); // Capture latency in milliseconds
        return tl + cl;

    }
}
