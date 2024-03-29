package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeRejectNote;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeContainer implements BaseContainer {
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private IntakeSubsystem intakeSubsystem;

    // private final CommandXboxController m_driverController =
    //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    public IntakeContainer() {
        intakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");
        configureBindings();
    }

    private void configureBindings() {

        //xboxController.x().onTrue(new IntakePickUpFromGround(intakeSubsystem));
        // xboxController.y().onTrue(new SpinIntakePID(intakeSubsystem, Constants.IntakeConstants.MAX_ARM_ANGLE));
        // xboxController.a().onTrue(new IntakePassNoteToPizzaBox(intakeSubsystem, null));
        
        xboxController.b().onTrue(new IntakeRejectNote(intakeSubsystem));
        
        // Command command = new SpinIntakePID(intakeController, intakeSubsystem, 0);
        // command = command.andThen(new IntakePickUpFromGround(intakeSubsystem));
        // command = command.andThen(new SpinIntakePID(intakeController, intakeSubsystem, Constants.IntakeConstants.MAX_ARM_ANGLE))c
        // command = command.andThen(new IntakePassNoteToPizzaBox(intakeSubsystem));

    }
}
