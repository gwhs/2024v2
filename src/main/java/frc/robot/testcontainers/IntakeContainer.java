package frc.robot.testcontainers;

import edu.wpi.first.math.controller.PIDController;
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
        xboxController.a().onTrue(new SpinIntakeArmMotor(intakeSubsystem, 77));
    }
}
