
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DashboardStore;
import frc.robot.util.CoralManagement.ElevatorPosition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

public class ElevatorSubsystem extends SubsystemBase {
    private static final double kP = 0.1;
    private static final double MAX_OUTPUT = 0.95;
    private static final double MIN_OUTPUT = -0.95;

    private static final int CAN_ID = 13;

    private static final double POSITION_THRESHOLD = 1.5;

    private static final double MAX_POS = 158;
    private static final double MIN_POS = 0.5;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_POS).reverseSoftLimit(MIN_POS)
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private static final LimitSwitchConfig LIMIT_SWITCH = new LimitSwitchConfig()
            .reverseLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchEnabled(false);

            private static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
            .p(kP)
            .maxOutput(MAX_OUTPUT)
            .minOutput(MIN_OUTPUT);

    private SparkBaseConfig CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true)
            .apply(SOFT_LIMITS)
            .apply(CLOSED_LOOP_CONFIG)
            .apply(LIMIT_SWITCH);

    private double m_target = 0;

    private final SparkFlex m_motor = new SparkFlex(CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();

    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public ElevatorSubsystem() {
        m_motor.configure(
                CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        setupDashboard();
        
        m_encoder.setPosition(0.0);

        // When the limit switch is tripped, reset encoder
        // new Trigger(this::limitSwitchTripped).onTrue(runOnce(() -> m_encoder.setPosition(0.0)));
    }

    private void setupDashboard() {
        DashboardStore.add("Elevator Target", targetSupplier());
        DashboardStore.add("Elevator Velocity", m_encoder::getVelocity);
        DashboardStore.add("Elevator Position", m_encoder::getPosition);
    }

    /** Vbus Commands */
    private void set(double speed) {
        m_motor.set(speed);
    }

    public Command runElevatorCommand(DoubleSupplier speed) {
        return runOnce(() -> set(speed.getAsDouble()));
    }

    public Command stopElevatorCommand() {
        return runElevatorCommand(() -> 0.0);
    }

    /** PID Commands */
    public Command runToPositionCommand(Supplier<ElevatorPosition> position) {
        return runOnce(() -> runToPosition(position.get()));
    }

    private void runToPosition(ElevatorPosition position) {
        m_target = position.Height;
        m_pid.setReference(m_target, ControlType.kPosition);
    }

    public Command holdPositionCommand() {
        return runOnce(() -> m_pid.setReference(m_encoder.getPosition(), ControlType.kPosition));
    }

    /** Encoder & Target */
    private double getPosition() {
        return m_encoder.getPosition();
    }

    public DoubleSupplier positionSupplier() {
        return this::getPosition;
    }

    private boolean inRange() {
        return Math.abs(m_target - getPosition()) < POSITION_THRESHOLD;
    }

    public BooleanSupplier inRangeSupplier() {
        return this::inRange;
    }

    public DoubleSupplier targetSupplier() {
        return () -> m_target;
    }

    /** Limit Switch */
    // private boolean limitSwitchTripped() {
    //     return m_motor.getReverseLimitSwitch().isPressed();
    // }
}