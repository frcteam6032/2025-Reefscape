
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorPosition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int CAN_ID = -1;

    private static final double INCHES_TO_ROT = -1;
    private static final double ROT_TO_INCHES = 1 / INCHES_TO_ROT;

    private static final double POSITION_THRESHOLD = 0.25;

    private static final double MAX_POS_INCHES = 53;
    private static final double MIN_POS_INCHES = 0;

    private static final SoftLimitConfig SOFT_LIMITS = new SoftLimitConfig()
            .forwardSoftLimit(MAX_POS_INCHES * INCHES_TO_ROT).reverseSoftLimit(MIN_POS_INCHES * INCHES_TO_ROT)
            .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    private static final SparkBaseConfig CONFIG = new SparkMaxConfig().idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(false)
            .apply(SOFT_LIMITS);

    private double m_target = 0;

    private final SparkFlex m_motor = new SparkFlex(CAN_ID, MotorType.kBrushless);
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public ElevatorSubsystem() {
        m_motor.configure(
                CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void set(double speed) {
        m_motor.set(speed);
    }

    public Command runElevatorCommand(double speed) {
        return runOnce(() -> set(speed));
    }

    public Command stopElevatorCommand() {
        return runElevatorCommand(0.0);
    }

    public Command runToPositionCommand(ElevatorPosition position) {
        return runOnce(() -> runToPosition(position));
    }

    private void runToPosition(ElevatorPosition position) {
        m_target = position.Height;
        m_pid.setReference(m_target * INCHES_TO_ROT, ControlType.kPosition);
    }

    private double getPosition() {
        return m_encoder.getPosition();
    }

    public DoubleSupplier positionSupplier() {
        return this::getPosition;
    }

    private boolean inRange() {
        return Math.abs(m_target - (getPosition() * ROT_TO_INCHES)) < POSITION_THRESHOLD;
    }

    public BooleanSupplier inRangeSupplier() {
        return this::inRange;
    }

}