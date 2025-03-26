
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CoralManagement.ElevatorPosition;
import frc.robot.util.DashboardStore;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralInfeed extends SubsystemBase {
    private static final double kP = 0.1;
    private static final double MAX_OUTPUT = 0.4;
    private static final double MIN_OUTPUT = -0.4;

    private static final int PIVOT_ID = 11;
    private static final int INTAKE_ID = 12;

    // 270 - 0
    private static final int MAX_ANGLE = 270;
    private static final int MIN_ANGLE = 5;

    // Degrees
    private static final double POSITION_THRESHOLD = 5;

    private static final double ROT_TO_DEG = 360 / 60;
    private static final double DEG_TO_ROT = 1 / ROT_TO_DEG;

    double lastIntakeVel = 0.0;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_ANGLE * DEG_TO_ROT).reverseSoftLimit(MIN_ANGLE * DEG_TO_ROT)
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
            .p(kP)
            .maxOutput(MAX_OUTPUT)
            .minOutput(MIN_OUTPUT);

    private static final SparkBaseConfig INTAKE_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(40);

    private final SparkBaseConfig PIVOT_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(40)
            .apply(CLOSED_LOOP_CONFIG)
            .apply(SOFT_LIMITS);

    private final SparkMax m_pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    private final SparkMax m_intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);

    private final SparkClosedLoopController m_pid = m_pivotMotor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_pivotMotor.getEncoder();

    private double m_target = 0;

    public CoralInfeed() {
        m_pivotMotor.configure(PIVOT_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(INTAKE_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_target = getPosition() * DEG_TO_ROT;

        setupDashboard();

        m_encoder.setPosition(0.0);
    }

    private void setupDashboard() {
        DashboardStore.add("Coral Pivot", () -> getPosition() * ROT_TO_DEG);
        DashboardStore.add("Coral Target", () -> m_target);
    }

    /** Pivot Commands */
    public Command runToPositionCommand(Supplier<ElevatorPosition> position) {
        return runOnce(() -> runToPosition(position.get()));
    }

    private void runToPosition(ElevatorPosition position) {
        m_target = position.Angle;
        m_pid.setReference(m_target * DEG_TO_ROT, ControlType.kPosition);
    }

    public Command holdPositionCommand() {
        return runOnce(() -> m_pid.setReference(m_encoder.getPosition(), ControlType.kPosition));
    }

    /** Encoder */
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

    /** Vbus Commands */
    public Command runPivotCommand(DoubleSupplier value) {
        return runOnce(() -> runPivot(value.getAsDouble()));
    }

    private void runPivot(double value) {
        m_pivotMotor.set(value);
    }

    public Command stopPivotCommand() {
        return runPivotCommand(() -> 0.0);
    }

    /* Intake Commands */
    public Command setIntakeCommand(double value) {
        return runOnce(() -> intake(value));
    }

    public Command scoreCommand() {
        return setIntakeCommand(-0.8);
    }

    public Command intakeCommand() {
        return setIntakeCommand(0.5);
    }

    public Command autoScoreCommand() {
        return scoreCommand().andThen(Commands.waitSeconds(0.5)).andThen(stopIntakeCommand());
    }

    public Command smartIntakeCommand() {
        return intakeCommand()
                .beforeStarting(() -> lastIntakeVel = 0.0)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(Commands.waitSeconds(5.0).until(this::hasPiece))
                .andThen(stopIntakeCommand());
    }

    private void intake(double value) {
        m_intakeMotor.set(value);
    }

    public Command stopIntakeCommand() {
        return setIntakeCommand(0.0);
    }

    @Override
    public void periodic() {
        hasPiece();
    }

    public boolean hasPiece() {
        double curVel = m_intakeMotor.getEncoder().getVelocity();
        double accel = curVel - lastIntakeVel;
        boolean slowingDown = accel < 0;
        lastIntakeVel = curVel;
        boolean speedLow = curVel < 10;
        boolean currentHigh = m_intakeMotor.getOutputCurrent() > 5.0;
        boolean on = Math.abs(m_intakeMotor.getAppliedOutput()) > 0.1;
        SmartDashboard.putNumber("Encoder Vel", curVel);
        SmartDashboard.putNumber("Current", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("On", m_intakeMotor.getAppliedOutput());

        SmartDashboard.putBoolean("bool accel", slowingDown);
        SmartDashboard.putBoolean("bool speed", speedLow);
        SmartDashboard.putBoolean("bool current", currentHigh);
        SmartDashboard.putBoolean("bool on", on);

        return speedLow && currentHigh && on && slowingDown;
    }
}