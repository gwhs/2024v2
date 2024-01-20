package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ledcommands.ChangeLEDToBlue;
import frc.robot.commands.ledcommands.ChangeLEDToGreen;
import frc.robot.commands.ledcommands.ChangeLEDToRed;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDContainer implements BaseContainer {
  
    private final CommandXboxController xboxController = new CommandXboxController(0);
    LEDSubsystem led = new LEDSubsystem(9);
    
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public LEDContainer() {
        configureBindings();
    }


    private void configureBindings() {
        xboxController.x().onTrue(new ChangeLEDToBlue(led));
        xboxController.y().onTrue(new ChangeLEDToRed(led));
        xboxController.a().onTrue(new ChangeLEDToGreen(led));
    }

}
