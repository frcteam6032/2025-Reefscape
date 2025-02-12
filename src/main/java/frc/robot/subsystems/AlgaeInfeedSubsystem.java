
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeInfeedSubsystem extends SubsystemBase {

    private SparkMax pivotMotor;
    private SparkMax intakeMotor;

    private final int PIVOT_MOTOR_ID = -1;
    private final int INTAKE_MOTOR_ID = -1;
    private final int maxAngle = -1;
    private final int minAngle = -1;
    private DutyCycleEncoder m_DutyCycleEncoder;


    public AlgaeInfeedSubsystem() {
        m_DutyCycleEncoder = new DutyCycleEncoder(0);
        pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        
        // TODO: move configs into constructors

        pivotMotor.configure(Configs.MAXSwerveModule.algaePivotMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        intakeMotor.configure(Configs.MAXSwerveModule.algaeIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

   
    public boolean verifyAngle() {
        return m_DutyCycleEncoder.get() > minAngle && m_DutyCycleEncoder.get() < maxAngle ? true : false;
    }

    // High precision acceleration control H.P.A.C
    public void set_speed_pivot(double value) {
       if (verifyAngle() == true) {
        pivotMotor.set(value);
       }
    }

   // TODO: do this for everything else 
    public Command stopPivotCommand() {
        return runOnce(() -> stop_pivot());
    }

    public void stop_pivot() {
        pivotMotor.set(0);
    }

    public void set_speed_intake(double value) {
        intakeMotor.set(value);
    }

    public void stop_intake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }
}