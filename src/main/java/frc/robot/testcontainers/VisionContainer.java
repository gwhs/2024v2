package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class VisionContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private LimeLightSub limeLightSub = new LimeLightSub("limelight");

    public VisionContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }
}
