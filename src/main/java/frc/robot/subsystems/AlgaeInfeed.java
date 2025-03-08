
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeInfeed extends SubsystemBase {
    private static final int PIVOT_ID = -1;
    private static final int INTAKE_ID = -1;

    private static final int MAX_ANGLE = -1;
    private static final int MIN_ANGLE = -1;

    private static final SparkBaseConfig INTAKE_CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);
    private static final SparkBaseConfig PIVOT_CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);

    private final SparkMax m_pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);;
    private final SparkMax m_intakeMotor = new SparkMax(INTAKE_ID, MotorType.kBrushless);

    private DutyCycleEncoder m_absoluteEncoder;

    public AlgaeInfeed() {
        m_absoluteEncoder = new DutyCycleEncoder(0);

        m_pivotMotor.configure(PIVOT_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(INTAKE_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Pivot Angle", m_absoluteEncoder.get());
        SmartDashboard.putNumber("Pivot Position", m_pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot Velocity", m_pivotMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Position", m_intakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Current", m_intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot Current", m_pivotMotor.getOutputCurrent());

    }

    private boolean inRange() {
        return m_absoluteEncoder.get() > MIN_ANGLE && m_absoluteEncoder.get() < MAX_ANGLE ? true : false;
    }

    public Command runPivotCommand(double value) {
        return runOnce(() -> runPivot(value));
    }

    public BooleanSupplier inRangeSupplier() {
        return this::inRange;
    }

    private void runPivot(double value) {
        if (inRangeSupplier().getAsBoolean()) {
            m_pivotMotor.set(value);
        }
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

}