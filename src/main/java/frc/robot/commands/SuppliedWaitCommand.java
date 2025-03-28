package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * A command that does nothing but takes a specified amount of time to finish.
 * Useful for
 * CommandGroups. Can also be subclassed to make a command with an internal
 * timer.
 *
 * <p>
 * This class is not provided by the NewCommands VendorDep
 */
public class SuppliedWaitCommand extends Command {
    protected Timer m_timer = new Timer();
    private double m_duration;
    private final DoubleSupplier m_durationSupplier;

    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the
     * specified duration.
     *
     * @param timeSupplier
     *                     the supplier of the time to wait, in seconds
     */
    public SuppliedWaitCommand(DoubleSupplier timeSupplier) {
        m_durationSupplier = timeSupplier;
        // SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_duration = m_durationSupplier.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("duration", () -> m_duration, null);
    }
}