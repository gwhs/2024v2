package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;

public class ArmContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public ArmContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }
}
