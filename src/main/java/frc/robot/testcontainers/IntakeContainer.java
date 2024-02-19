package frc.robot.testcontainers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommands.SpinIntakePID;
import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
import frc.robot.commands.IntakeCommands.IntakeRejectNote;
import frc.robot.commands.IntakeCommands.SpinIntakeArmMotor;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeContainer implements BaseContainer {

    private final CommandXboxController xboxController = new CommandXboxController(0);
    private IntakeSubsystem intakeSubsystem;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public IntakeContainer() {
        intakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");
        configureBindings();
    }

    private void configureBindings() {
        //intakeSubsystem.enable();

        xboxController.a().onTrue(Commands.runOnce(() -> {
            intakeSubsystem.setIntakeArmAngle(0); 
            intakeSubsystem.enable();}, intakeSubsystem));
        

        // xboxController.x().onTrue(new SpinIntakeArmMotor(intakeSubsystem, 0));
        // xboxController.y().onTrue(new SpinIntakeArmMotor(intakeSubsystem, Constants.IntakeConstants.MAX_ARM_ANGLE));

        //xboxController.a().onTrue(new IntakeRejectNote(intakeSubsystem));
        //xboxController.b().onTrue(new IntakePickUpFromGround(intakeSubsystem));
        
        // xboxController.x().onTrue(new SpinIntakePID(intakeController, intakeSubsystem, 0));
        // xboxController.y().onTrue(new SpinIntakePID(intakeController, intakeSubsystem, 106));

        // xboxController.a().onTrue(new IntakePickUpFromGround(intakeSubsystem));
        // xboxController.b().onTrue(new IntakePassNoteToPizzaBox(intakeSubsystem));


        // Command command = new SpinIntakePID(intakeController, intakeSubsystem, 0);
        // command = command.andThen(new IntakePickUpFromGround(intakeSubsystem));
        // command = command.andThen(new SpinIntakePID(intakeController, intakeSubsystem, Constants.IntakeConstants.MAX_ARM_ANGLE))c
        // command = command.andThen(new IntakePassNoteToPizzaBox(intakeSubsystem));

    }
}
