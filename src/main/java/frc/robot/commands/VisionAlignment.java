package frc.robot.commands;

import java.time.OffsetDateTime;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignment extends Command  {
    DriveSubsystem m_driveSubsystem;
    DoubleSupplier m_speedX;
    DoubleSupplier m_speedY;
    DoubleSupplier m_offset;
    BooleanSupplier m_targetFound;
    DoubleSupplier m_joyAngle;

  /** Creates a new RotateToAngle. */
  public VisionAlignment(DriveSubsystem driveTrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier offset, BooleanSupplier targetFound, DoubleSupplier joyAngle) {
    this.m_driveSubsystem = driveTrain;
    this.m_speedX = xSpeed;
    this.m_speedY = ySpeed;
    this.m_offset = offset;
    this.m_targetFound = targetFound;
    this.m_joyAngle = joyAngle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.setYawControllerSetpoint(m_driveSubsystem.getHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_targetFound.getAsBoolean() == true) {
        m_driveSubsystem.setYawControllerSetpoint(m_driveSubsystem.getHeading() + m_offset.getAsDouble());
    }
    else {
        m_driveSubsystem.setPointRelative(m_joyAngle.getAsDouble());
    }
    m_driveSubsystem.drivePIDHeading(m_speedX.getAsDouble(), m_speedY.getAsDouble(), true);
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
