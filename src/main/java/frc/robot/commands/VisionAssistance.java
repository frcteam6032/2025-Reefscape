package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class VisionAssistance extends Command {
    private final DriveSubsystem m_drive;

    public VisionAssistance(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Enable alignment 
        m_drive.enableVisionAlign(true);
    }

    @Override
    public void execute() {
        // Logic is handled in the drive subsystem
    }

    @Override
    public void end(boolean interrupted) {
        // Disable alignment 
        m_drive.enableVisionAlign(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
