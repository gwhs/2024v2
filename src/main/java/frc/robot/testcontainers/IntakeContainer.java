package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommands.LowerArmIntake;
import frc.robot.commands.IntakeCommands.StartIntake;
import frc.robot.commands.IntakeCommands.UpperArmIntake;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeContainer implements BaseContainer {

    private final CommandXboxController xboxController = new CommandXboxController(0);
    private IntakeSubsystem IntakeSubsystem;
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public IntakeContainer() {
        IntakeSubsystem = new IntakeSubsystem(0, 0, 0, 0, 0, 0, null);
        configureBindings();
    }

    private void configureBindings() {
        xboxController.x().onTrue(new SequentialCommandGroup(
            new LowerArmIntake(IntakeSubsystem, 0), 
            new UpperArmIntake(IntakeSubsystem)));
        xboxController.a().onTrue(new LowerArmIntake(IntakeSubsystem, 0));
        xboxController.b().onTrue(new UpperArmIntake(IntakeSubsystem));

        StartIntake intake = new StartIntake(IntakeSubsystem);
        xboxController.y().onTrue(intake);
    }
}
