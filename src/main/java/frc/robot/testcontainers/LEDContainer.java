package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public LEDContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }

    LEDSubsystem led = new LEDSubsystem();
    
}
