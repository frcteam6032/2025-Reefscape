
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ExampleMotor extends SubsystemBase {
    private static final int MOTOR_ID = 0; // Set this to what we set it to in the rev hardware ID app

    private static final SparkBaseConfig MOTOR_CONFIG = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(50);



    private final SparkMax m_Motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);


    public ExampleMotor() {
    
        m_Motor.configure(MOTOR_CONFIG, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // Create spin command using Command
    // Create stop commandusing Command
    // Set low level motor speed command

}