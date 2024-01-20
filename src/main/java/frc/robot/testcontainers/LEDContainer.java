package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDContainer implements BaseContainer {
  
    private final XboxController xboxController = new XboxController(0);
    LEDSubsystem led = new LEDSubsystem(9);
    
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public LEDContainer() {
        configureBindings();
    }


    private void configureBindings() {}

    if ()

}
