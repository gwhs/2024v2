package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;

public class DriveContainer implements BaseContainer {

    // todo: add the Drivetrain subsystem

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public DriveContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }    
}
