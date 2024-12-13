/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.
   Open Source Software - may be modified and shared by FRC teams. The code
   must be accompanied by the FIRST BSD license file in the root directory of
   the project.
   ----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private double tx;
  private double ty;
  private double tv;
  private double[] botPose = new double[6];
  private double latency; // In milliseconds

  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0);
    tv = m_limelightTable.getEntry("tv").getDouble(0);

    // Get robot pose from limelight
    botPose = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    // Get latency to help determine the correct odometry record
    double tl = m_limelightTable.getEntry("tl").getDouble(0); // Pipeline latency in milliseconds
    double cl = m_limelightTable.getEntry("cl").getDouble(0); // Capture latency in milliseconds
    latency = tl + cl;
  }

  /**
   Get horzontial offet in degrees
   */
  public double getTX() {
    return tx;
  }

  /**
   Get vertical offset in degrees
   */
  public double getTY() {
    return ty;
  }

  /**
   See if a target is found 
   */
  // Remember that the target can only be seen -29.8 to 29.8 degrees in any axis
  public boolean isTargetValid() {
    return (tv == 1.0);
  }

  /**
   Estimates in order: x, y, z, roll, pitch, yaw
   */
  public double[] getBotPose() {
    return botPose;
  }

  /**
   Latency in ms
   */
  public double getLatency() {
    return latency;
  }
}
