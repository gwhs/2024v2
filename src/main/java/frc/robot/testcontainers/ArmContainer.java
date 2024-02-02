package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.SpinNoteContainerMotor;
import frc.robot.commands.StopNoteContainerMotor;
import frc.robot.commands.SwingBack;
import frc.robot.commands.SwingForward;


public class ArmContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public ArmContainer() {
        configureBindings();

    }


    private void configureBindings() {
       xboxController.x().onTrue(new );
        
    }
}
