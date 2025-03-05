
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPosition;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class CoralInfeed extends SubsystemBase {
    private static final int PIVOT_ID = -1;
    private static final int INTAKE_ID = -1;

    private static final int MAX_ANGLE = -1;
    private static final int MIN_ANGLE = -1;

    private static final double POSITION_THRESHOLD = 2;

    private static final double ROT_TO_DEG = 360 / 3;
    private static final double DEG_TO_ROT = 1 / ROT_TO_DEG;
    private static final double ABS_REL = 3;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_ANGLE * DEG_TO_ROT).reverseSoftLimit(MIN_ANGLE * DEG_TO_ROT)
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private static final SparkBaseConfig INTAKE_CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);
    private static final SparkBaseConfig PIVOT_CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false)
            .apply(SOFT_LIMITS);

    private final SparkMax m_pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    private final SparkMax m_intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);

    private final SparkClosedLoopController m_pid = m_pivotMotor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_pivotMotor.getEncoder();

    private double m_target = 0;

    private DutyCycleEncoder m_absoluteEncoder;

    public CoralInfeed() {
        m_absoluteEncoder = new DutyCycleEncoder(0);

        m_pivotMotor.configure(PIVOT_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(INTAKE_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command runPivotCommand(double value) {
        return runOnce(() -> runPivot(value));
    }

    private void runPivot(double value) {
        m_pivotMotor.set(value);
    }

    public Command stopPivotCommand() {
        return runPivotCommand(0.0);
    }

    public Command intakeCommand(double value) {
        return runOnce(() -> intake(value));
    }

    private void intake(double value) {
        m_intakeMotor.set(value);
    }

    public Command stopIntakeCommand() {
        return intakeCommand(0.0);
    }

    public Command runToPositionCommand(ElevatorPosition position) {
        return runOnce(() -> runToPosition(position));
    }

    private void runToPosition(ElevatorPosition position) {
        m_target = position.Angle;
        m_pid.setReference(m_target * DEG_TO_ROT, ControlType.kPosition);
    }

    private double getPosition() {
        return m_encoder.getPosition();
    }

    public DoubleSupplier positionSupplier() {
        return this::getPosition;
    }

    private boolean inRange() {
        return Math.abs(m_target - (getPosition() * ROT_TO_DEG)) < POSITION_THRESHOLD;
    }

    public BooleanSupplier inRangeSupplier() {
        return this::inRange;
    }

    public void resetEncoder() {
        m_encoder.setPosition(m_absoluteEncoder.get() * ABS_REL);
    }
}