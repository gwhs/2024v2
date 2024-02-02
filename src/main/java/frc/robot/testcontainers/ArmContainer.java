package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        ArmSubsystem arm = new ArmSubsystem(0, "testCan", 0, "testCan", 0, 0); 

    public ArmContainer() {
        configureBindings();

    }


    private void configureBindings() {
       m_driverController.x().onTrue(new SpinNoteContainerMotor(arm, .001, .001));
        
    }
}
