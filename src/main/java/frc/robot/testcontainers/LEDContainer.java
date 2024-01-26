package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ledcommands.ChangeLEDColor;
import frc.robot.commands.ledcommands.ChangeLEDToBlue;
import frc.robot.commands.ledcommands.ChangeLEDToGreen;
import frc.robot.commands.ledcommands.ChangeLEDToRed;
import frc.robot.subsystems.LEDSubsystem;

public class LEDContainer implements BaseContainer {

  private static final int ledPortNumber = 9;
    private final CommandXboxController xboxController = new CommandXboxController(0);

    LEDSubsystem led = new LEDSubsystem(ledPortNumber);
    
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public LEDContainer() {
        configureBindings();
    }


    private void configureBindings() {
        xboxController.x().onTrue(new ChangeLEDToBlue(led));//pressing x on the controller runs a
        xboxController.y().onTrue(new ChangeLEDToRed(led));
        xboxController.b().onTrue(new ChangeLEDToGreen(led));
        xboxController.a().onTrue(new ChangeLEDColor(led, 255, 0, 255));
        xboxController.rightBumper().onTrue(new ChangeLEDColor(led, 0, 0, 0));
    }


    private Command ChangeLEDColor(LEDSubsystem led2, int i, int j, int k) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'ChangeLEDColor'");
    }

}
