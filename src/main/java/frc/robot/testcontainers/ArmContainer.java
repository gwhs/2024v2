package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.IntakeCommands.PickUpFromGroundAndPassToPizzaBox;

public class ArmContainer implements BaseContainer {

  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        //CAN_Network
        ArmSubsystem arm = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "CAN_Network", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        // PizzaBoxSubsystem pizzaBox = new PizzaBoxSubsystem(PizzaBoxSubsystem.PizzaBox.PIZZABOX_ID, 
        //             "rio", PizzaBoxSubsystem.PizzaBox.SERVO_PWN_SLOT);
                        
        private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");
        

    public ArmContainer() {
        configureBindings();
    }

    private void configureBindings() {

        arm.targetArmAngle(arm.encoderGetAngle());
        arm.enable();
        
//Pick Up and Put in PizzaBox Command
        // m_driverController.a().onTrue(new PickUpFromGroundAndPassToPizzaBox(pizzaBox, arm, intakeSubsystem));
        // m_driverController.x().whileTrue(new SpinNoteContainerMotor(pizzaBox, 400, 100));

    }

}
