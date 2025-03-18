
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.DashboardStore;
import frc.robot.util.CoralManagement.ElevatorPosition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
    /* TMP: PID Constants */
    private boolean enablePIDTuning = true;

    private double kP = 0.1;
    private double kD = 0.0;
    private double kFF = 0.0;
    private double kMaxOutput = 0.3;
    private double kMinOutput = -0.3;

    private static final int CAN_ID = 13;

    private static final double POSITION_THRESHOLD = 1.5;

    // gonna do this stuff in rotations
    private static final double MAX_POS = -1;
    private static final double MIN_POS = -1;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_POS).reverseSoftLimit(MIN_POS)
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private static final LimitSwitchConfig LIMIT_SWITCH = new LimitSwitchConfig()
            .reverseLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchEnabled(false);

    private ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    private SparkBaseConfig CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true)
            // .apply(SOFT_LIMITS)
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

        if (enablePIDTuning) {
            // display PID coefficients on SmartDashboard
            SmartDashboard.putNumber("Elevator P", kP);
            SmartDashboard.putNumber("Elevator D", kD);
            SmartDashboard.putNumber("Elevator FF", kFF);
            SmartDashboard.putNumber("Elevator Max", kMaxOutput);
            SmartDashboard.putNumber("Elevator Min", kMinOutput);

            reapplyPID();
        }

        m_encoder.setPosition(0.0);

        // When the limit switch is tripped, reset encoder
        // new Trigger(this::limitSwitchTripped).onTrue(runOnce(() -> m_encoder.setPosition(0.0)));
    }

    private void reapplyPID() {
        closedLoopConfig.pidf(kP, 0.0, kD, kFF);
        closedLoopConfig.maxOutput(kMaxOutput);
        closedLoopConfig.minOutput(kMinOutput);

        CONFIG.apply(closedLoopConfig);
        m_motor.configure(
                CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
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

    public Command runElevatorCommand(double speed) {
        return runOnce(() -> set(speed));
    }

    public Command stopElevatorCommand() {
        return runElevatorCommand(0.0);
    }

    /** PID Commands */
    public Command runToPositionCommand(ElevatorPosition position) {
        return runOnce(() -> runToPosition(position));
    }

    private void runToPosition(ElevatorPosition position) {
        m_target = position.Height;
        m_pid.setReference(m_target, ControlType.kPosition);
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
    private boolean limitSwitchTripped() {
        return m_motor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public void periodic() {
        if (enablePIDTuning) {
            double p = SmartDashboard.getNumber("Elevator P", kP);
            double d = SmartDashboard.getNumber("Elevator D", kD);
            double ff = SmartDashboard.getNumber("Elevator FF", kFF);
            double max = SmartDashboard.getNumber("Elevator Max", kMaxOutput);
            double min = SmartDashboard.getNumber("Elevator Min", kMinOutput);

            // if PID coefficients on SmartDashboard have changed, write new values to
            // controller
            if ((p != kP)) {
                kP = p;
                reapplyPID();
            }
            if ((d != kD)) {
                kD = d;
                reapplyPID();
            }
            if ((ff != kFF)) {
                kFF = ff;
                reapplyPID();
            }
            if ((max != kMaxOutput) || (min != kMinOutput)) {
                kMinOutput = min;
                kMaxOutput = max;
                reapplyPID();
            }
        }
    }
}