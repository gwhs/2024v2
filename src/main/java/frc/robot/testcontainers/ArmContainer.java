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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.StopNoteContainerMotor;

import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import frc.robot.commands.Arm.SpinAndSwing;
import frc.robot.commands.Arm.SpinArmPID;
import frc.robot.commands.Arm.SpinToArmAngle;

import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;



public class ArmContainer implements BaseContainer {

  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        //CAN_Network
        ArmSubsystem arm = new ArmSubsystem(Constants.Arm.ARM_ID, "CAN_Network", Constants.Arm.PIZZABOX_ID, "rio", 
                        Constants.Arm.ENCODER_DIO_SLOT, Constants.Arm.SERVO_PWN_SLOT); 

    public ArmContainer() {
        configureBindings();

    }

    private void configureBindings() {
        //m_driverController.x().onTrue(new SwingForwardServo(arm));

       m_driverController.y().onTrue(Commands.runOnce(() -> {
        arm.setGoal(40+ arm.getMeasurement());
        }, arm));

        m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            arm.targetArmAngle(30);}));

        
       m_driverController.rightBumper().onTrue(new SpinToArmAngle(arm, 0));
       m_driverController.x().onTrue(new SpinToArmAngle(arm, arm.encoderGetAngle() + 10));

        //m_driverController.a().onTrue(new SpinAndSwing(arm));

//We might not need this anymore (2/10/24)
        Shuffleboard.getTab("Arm").addDouble("encoder",()->arm.encoderGetAngle());
        Shuffleboard.getTab("Arm").addDouble("arm",()->arm.getArmAngle());

        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
