package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PizzaBoxSubsystem;


import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.StopNoteContainerMotor;

import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import frc.robot.commands.Arm.SpinAndSwing;
import frc.robot.commands.Arm.SpinToArmAngle;

import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
import frc.robot.commands.IntakeCommands.SpinIntakePID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.controller.PIDController;





public class ArmContainer implements BaseContainer {

  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        //CAN_Network
        ArmSubsystem arm = new ArmSubsystem(ArmSubsystem.Arm.ARM_ID, "CAN_Network", 
                        ArmSubsystem.Arm.ENCODER_DIO_SLOT);

        PizzaBoxSubsystem pizzaBox = new PizzaBoxSubsystem(PizzaBoxSubsystem.PizzaBox.PIZZABOX_ID, 
                    "rio", PizzaBoxSubsystem.PizzaBox.SERVO_PWN_SLOT);
                        
        private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(Constants.IntakeConstants.INTAKE_LOWER_INTAKE_ID,Constants.IntakeConstants.INTAKE_SPIN_MOTOR_ID, "rio");
        

    public ArmContainer() {
        configureBindings();
    }

    private void configureBindings() {

        arm.targetArmAngle(ArmSubsystem.Arm.INTAKE_ANGLE);
        arm.enable();
        
//Pick Up and Put in PizzaBox Command
        m_driverController.a().onTrue(new SpinIntakePID(intakeSubsystem, 0).
                andThen(new IntakePickUpFromGround(intakeSubsystem)).
                andThen(new SpinIntakePID(intakeSubsystem, 75)).
                andThen((Commands.runOnce(() -> {
                    arm.targetArmAngle(ArmSubsystem.Arm.INTAKE_ANGLE);
                    }, arm))).
                andThen(new IntakePassNoteToPizzaBox(intakeSubsystem, pizzaBox)
                ));

        m_driverController.y().onTrue(new SpinAndSwing(pizzaBox, arm, 245));

        m_driverController.x().whileTrue(new SpinNoteContainerMotor(pizzaBox, 400, 100));
        // xboxController.b().onTrue(new IntakePassNoteToPizzaBox(intakeSubsystem));




        m_driverController.start().onTrue(Commands.runOnce(()-> {
            if(arm.isEnabled())
            {
                arm.disable();
            }
            else
            {
                arm.enable();
            }
        }, arm));

        // m_driverController.a().onTrue(Commands.runOnce(() -> {
        //     arm.targetArmAngle(ArmSubsystem.Arm.SPEAKER_LOW_ANGLE);
        //     }, arm));

        // m_driverController.back().onTrue(new SpinToArmAngle(arm, ArmSubsystem.Arm.CLIMBING_ANGLE));

        // m_driverController.b().onTrue(new SpinToArmAngle(arm, 300));

    //    m_driverController.x().onTrue(new SpinToArmAngle(arm, 150));

    //    m_driverController.y().onTrue(Commands.runOnce(() -> {
    //     arm.targetArmAngle(ArmSubsystem.Arm.SPEAKER_HIGH_ANGLE);
    //     }, arm));

        m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            arm.targetArmAngle(ArmSubsystem.Arm.INTAKE_ANGLE);}));

        
    //    m_driverController.rightBumper().onTrue(new SwingForwardServo(pizzaBox));

        // m_driverController.x().onTrue(new SwingForwardServo(pizzaBox));
        // m_driverController.y().onTrue(new SwingBackServo(pizzaBox));


        // m_driverController.a().onTrue(new SpinAndSwing(pizzaBox, arm, 270));

        Shuffleboard.getTab("Arm").addDouble("encoder",()->arm.encoderGetAngle());
        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
