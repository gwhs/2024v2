package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommands.LowerArmIntake;
import frc.robot.commands.IntakeCommands.SpinIntakePID;
import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
import frc.robot.commands.IntakeCommands.UpperArmIntake;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeContainer implements BaseContainer {

    private final CommandXboxController xboxController = new CommandXboxController(0);
    private IntakeSubsystem intakeSubsystem;
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public IntakeContainer() {
        intakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, 0, "rio");
        configureBindings();
    }

    private void configureBindings() {

        xboxController.x().onTrue(new SpinIntakePID(intakeSubsystem, 0));
        //xboxController.a().onTrue(new SpinIntakePID(intakeSubsystem, 90));
        
        // xboxController.a().onTrue(new LowerArmIntake(intakeSubsystem, 10)); //b
        // xboxController.x().onTrue(new UpperArmIntake(intakeSubsystem)); //x
        // xboxController.y().onTrue(new IntakePickUpFromGround(intakeSubsystem));

        // press b - goes to 0, press x - goes to 90 , for controller labled BROKEN

    }
}
