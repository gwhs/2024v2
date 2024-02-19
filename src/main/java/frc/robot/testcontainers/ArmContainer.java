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
import frc.robot.commands.Arm.SwingBack;
import frc.robot.commands.Arm.SpinArm;

import frc.robot.commands.Arm.SwingForward;
import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import frc.robot.commands.Arm.SpinAndSwing;
import frc.robot.commands.Arm.SpinArmPID;
import frc.robot.commands.Arm.SpinToArmAngle;

import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
import frc.robot.commands.IntakeCommands.LowerArmIntake;
import frc.robot.commands.IntakeCommands.UpperArmIntake;

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
        ArmSubsystem arm = new ArmSubsystem(Constants.Arm.ARM_ID, "CAN_Network", Constants.Arm.PIZZABOX_ID, "rio", 0, 0); 
        //IntakeSubsystem intake = new IntakeSubsystem(0, 0, 1, 2, 3, "rio");
//  public IntakeSubsystem(int lowerIntakeId, int spinIntakeId, int channel1, int channel2, int channel3, String can)  {

        //Testing Purposes


        //Testing End

    public ArmContainer() {
        configureBindings();

    }


    private void configureBindings() {
    //    m_driverController.a().onTrue(new SwingForward(arm, 270, 10, 10, .25));
    //    m_driverController.b().onTrue(new SwingBack(arm, 10, 10, .25));
         m_driverController.x().onTrue(new SwingServo(arm));

       //SpinNoteContainerMotor army = new SpinNoteContainerMotor (arm, 0.25, 10);
       //m_driverController.y().onTrue(army);
       //m_driverController.x().onTrue(new StopNoteContainerMotor(arm));
       

       m_driverController.y().onTrue(Commands.runOnce(() -> {
        arm.setGoal(40+ arm.getMeasurement());
        }, arm));

        m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            arm.targetArmAngle(30);}));

        
       m_driverController.rightBumper().onTrue(new SpinToArmAngle(arm, 0));
       m_driverController.x().onTrue(new SpinToArmAngle(arm, arm.encoderGetAngle() + 10));

//We might not need this anymore (2/10/24)
        Shuffleboard.getTab("Arm").addDouble("encoder",()->arm.encoderGetAngle());
        Shuffleboard.getTab("Arm").addDouble("arm",()->arm.getArmAngle());

        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
