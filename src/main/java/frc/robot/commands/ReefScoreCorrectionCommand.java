package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.Limelight;

public class ReefScoreCorrectionCommand extends Command {

    private double backupVectorX = 0;
    private double backupVectorY = 0;

    private final DriveSubsystem m_driveSubsystem;
    private final Limelight m_limelight;

    private PIDController controller = new PIDController(DriveSubsystem.ROTATE_kP, 0.0, DriveSubsystem.ROTATE_kD);

    public ReefScoreCorrectionCommand(DriveSubsystem driveSubsystem, Limelight limelight) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_limelight = limelight;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        backupVectorX = 1;
        backupVectorY = 0;
    }
    // Called every time the scheduler runs while the command is scheduled.

    // This is our accurate automatic intake system A.A.I.S
    @Override
    public void execute() {

        // @SuppressWarnings("resource")
        controller.enableContinuousInput(-180, 180);

        double currentAngle = m_driveSubsystem.getHeading();

        double nearestMultipleOf60 = Math.round(currentAngle / 60.0) * 60.0;

        controller.setSetpoint(nearestMultipleOf60);

        double rotationOutput = controller.calculate(currentAngle);
        // Normalize
        double rotationCommand = rotationOutput / DriveConstants.kMaxAngularSpeed;

        if (m_limelight.isTargetValid()) {
            double offset = -m_limelight.getTX();
            // We need to move where the center of the target is
            // We have the TX and we need to solve the triangle (distance)
            double xComponent = (Math.cos(Math.toRadians(offset)) * m_limelight.getDistance());
            // IN CM
            double yComponent = (Math.sin(Math.toRadians(offset)) * m_limelight.getDistance()) + 0.2;

            double magnitude = Math.sqrt(Math.pow(xComponent, 2) + Math.pow(yComponent, 2));

            double strafeSpeedX = xComponent / magnitude;
            double strafeSpeedY = yComponent / magnitude;

            strafeSpeedX = 0.1 * Math.signum(xComponent);
            strafeSpeedY = 0.1 * Math.signum(xComponent);

            backupVectorX = strafeSpeedX;
            backupVectorY = strafeSpeedY;

            // If the offset is large = strafe
            if (Math.abs(offset) > 5) {

                m_driveSubsystem.joystickDrive(
                        strafeSpeedX,
                        strafeSpeedY,
                        rotationCommand,
                        false);
            } else {
                // Drive slow if small offset
                m_driveSubsystem.joystickDrive(
                        0.1,
                        0.0,
                        rotationCommand,
                        false);
            }
        }

        else {

            double strafeSpeedX;
            double strafeSpeedY;

            strafeSpeedX = backupVectorX;
            strafeSpeedY = backupVectorY;

            // If the offset is large = strafe

            m_driveSubsystem.joystickDrive(
                    -strafeSpeedX,
                    strafeSpeedY,
                    rotationCommand,
                    false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.reset();
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

}