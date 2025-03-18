
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DashboardStore;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeInfeed extends SubsystemBase {
    /* TMP: PID Constants */
    private boolean enablePIDTuning = false;

    private double kP = 0.1;
    private double kD = 0.0;
    private double kFF = 0.0;
    private double kMaxOutput = 0.3;
    private double kMinOutput = -0.3;

    private static final int PIVOT_ID = 9;
    private static final int INTAKE_ID = 10;

    private static final double MAX_ANGLE = 0.2;
    private static final double MIN_ANGLE = -30;

    private static final double STOW_ANGLE = -3;
    private static final double DEPLOY_ANGLE = -20;

    // Degrees
    private static final double POSITION_THRESHOLD = 5;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_ANGLE).reverseSoftLimit(MIN_ANGLE )
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

    private static final SparkBaseConfig INTAKE_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(20);

    private SparkBaseConfig PIVOT_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(30);
    // .apply(SOFT_LIMITS);

    private final SparkMax m_pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    private final SparkMax m_intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);

    private final SparkClosedLoopController m_pid = m_pivotMotor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_pivotMotor.getEncoder();

    private double m_target = 0;

    public AlgaeInfeed() {
        m_pivotMotor.configure(PIVOT_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(INTAKE_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_target = getPosition();

        setupDashboard();

        if (enablePIDTuning) {
            // display PID coefficients on SmartDashboard
            SmartDashboard.putNumber("Algae P", kP);
            SmartDashboard.putNumber("Algae D", kD);
            SmartDashboard.putNumber("Algae FF", kFF);
            SmartDashboard.putNumber("Algae Max", kMaxOutput);
            SmartDashboard.putNumber("Algae Min", kMinOutput);
            SmartDashboard.putNumber("Algae Target", m_target);
        }

        reapplyPID();

        m_encoder.setPosition(0.0);
    }

    private void reapplyPID() {
        closedLoopConfig.pidf(kP, 0.0, kD, kFF);
        closedLoopConfig.maxOutput(kMaxOutput);
        closedLoopConfig.minOutput(kMinOutput);

        PIVOT_CONFIG.apply(closedLoopConfig);
        m_pivotMotor.configure(PIVOT_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void setupDashboard() {
        DashboardStore.add("Algae Pivot", this::getPosition);
        DashboardStore.add("Algae Target", () -> m_target);
    }

    /** Pivot Commands */
    public Command deployCommand() {
        return runToAngleCommand(DEPLOY_ANGLE);
    }

    public Command stowCommand() {
        return runToAngleCommand(STOW_ANGLE);
    }

    public Command runToAngleCommand(double angle) {
        return runOnce(() -> runToPosition(angle));
    }

    private void runToPosition(double angle) {
        m_target = angle;
        m_pid.setReference(m_target, ControlType.kPosition);
    }

    /** Vbus Commands */
    public Command runPivotCommand(double value) {
        return runOnce(() -> runPivot(value));
    }

    private void runPivot(double value) {
        m_pivotMotor.set(value);
    }

    public Command stopPivotCommand() {
        return runPivotCommand(0.0);
    }

    public Command toggleCommand() {
        return Commands.either(
            runToAngleCommand(DEPLOY_ANGLE),
            runToAngleCommand(STOW_ANGLE),
            () -> m_target == STOW_ANGLE
        );
    }

    /** Encoder Stuff */
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

    /** Intake Commands */

    public Command intakeCommand(double value) {
        return runOnce(() -> intake(value));
    }

    private void intake(double value) {
        m_intakeMotor.set(value);
    }

    public Command stopIntakeCommand() {
        return intakeCommand(0.0);
    }

    @Override
    public void periodic() {
        if (enablePIDTuning) {

            double p = SmartDashboard.getNumber("Algae P", kP);
            double d = SmartDashboard.getNumber("Algae D", kD);
            double ff = SmartDashboard.getNumber("Algae FF", kFF);
            double max = SmartDashboard.getNumber("Algae Max", kMaxOutput);
            double min = SmartDashboard.getNumber("Algae Min", kMinOutput);

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

            // double target = SmartDashboard.getNumber("Algae Target", m_target);
            // m_pid.setReference(target, ControlType.kPosition);
        }
    }
}