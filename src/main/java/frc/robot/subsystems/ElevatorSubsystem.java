
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorSubsystem extends SubsystemBase {

    private final int ELEVATOR_MOTOR_ID = -1;

    // Coral scoring has an elevator height and different angles 
    public enum ElevatorPosition {
        FeederStation(-1, -1),
        Level1(-1,-1),
        Level2(41.8, 200),
        Level3(57.3, 200),
        Level4(-1, -1);
        // Height: Inches, Agle: Degrees
        ElevatorPosition(double height, double angle) {};
    }

    
    private SparkFlex elevatorMotor = new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  

    public ElevatorSubsystem() {
        elevatorMotor.configure(
            Configs.MAXSwerveModule.elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
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