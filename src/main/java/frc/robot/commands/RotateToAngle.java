// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
    DriveSubsystem m_driveSubsystem;
    DoubleSupplier m_speedX;
    DoubleSupplier m_speedY;
    DoubleSupplier m_offset;
    PIDController yawController;
  /** Creates a new RotateToAngle. */
  public RotateToAngle(DriveSubsystem driveTrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier offset) {
    this.m_driveSubsystem = driveTrain;
    this.m_speedX = xSpeed;
    this.m_speedY = ySpeed;
    this.m_offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawController = new PIDController(0.2, 0, 0);

    yawController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double output = yawController.calculate(m_offset.getAsDouble());
   m_driveSubsystem.joystickDrive(m_speedX.getAsDouble(), m_speedY.getAsDouble(), output / DriveConstants.kMaxAngularSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
