package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ExampleMotor;


public class RobotContainer {
    // Here create an instance of the class that will contain the subsystem to move the motor ---

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    public RobotContainer() {
        // Configure the buttons & default commands
        configureButtonBindings();  
    }



    /**
     * DONT CHANGE THESE
     */
    private void configureButtonBindings() {
        // Here configure the button binding for the motor
        
    }

    

}