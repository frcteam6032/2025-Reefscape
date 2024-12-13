// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Robot methods setup 
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // Camera setup
  private UsbCamera camera = CameraServer.startAutomaticCapture(0);
  // Set up main tag for driver/operator 
  private ShuffleboardTab tab_competition = Shuffleboard.getTab("Competition");
  // Odometry visualization setup 
  private Field2d field2d = new Field2d();
  // LimeLight debug
  GenericEntry targetFound = tab_competition.add("(LimeLight): Target Focused", false).withSize(4,4).getEntry();
  GenericEntry offset_x = tab_competition.add("(LimeLight): Inaccuracy [X]", 0).withSize(4,4).getEntry();
  GenericEntry display_yaw = tab_competition.add("YAW", 0).withSize(4,4).getEntry();


  public void camera_setup() {
    camera.setResolution(160, 120);
    tab_competition.add(camera).withSize(6, 4);
  }

  public void shuffleboard_field_setup() {
    SmartDashboard.putData("Field Pose", field2d);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and set up helper functions

    m_robotContainer = new RobotContainer();
    // Create and add the camera to the shuffleboard
    camera_setup();
    shuffleboard_field_setup();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

   public void update_shuffleboard_odometry_map() {

    Pose2d robotPose = m_robotContainer.update_field();
    field2d.setRobotPose(robotPose);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    update_shuffleboard_odometry_map();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void update_vision_display() {
    targetFound.setBoolean(m_robotContainer.targetValid());
    offset_x.setDouble(m_robotContainer.tx());
    offset_y.setDouble(m_robotContainer.ty());
    display_yaw.setDouble(m_robotContainer.get_display_yaw());
  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    update_vision_display();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
