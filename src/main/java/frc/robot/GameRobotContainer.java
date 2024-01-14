package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

public class GameRobotContainer implements BaseContainer {

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public GameRobotContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }
}

