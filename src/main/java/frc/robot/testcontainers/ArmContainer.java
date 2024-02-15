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
import frc.robot.commands.Arm.SwingForward;
import frc.robot.commands.Arm.SwingForwardServo;
import frc.robot.commands.Arm.SwingBackServo;
import frc.robot.commands.Arm.SpinAndSwing;


import frc.robot.commands.IntakeCommands.IntakePassNoteToPizzaBox;
import frc.robot.commands.IntakeCommands.IntakePickUpFromGround;
import frc.robot.commands.IntakeCommands.LowerArmIntake;
import frc.robot.commands.IntakeCommands.UpperArmIntake;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class ArmContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        //CAN_Network
        ArmSubsystem arm = new ArmSubsystem(Constants.Arm.ARM_ID, "CAN_Network", 3, "rio", 0, 0); 
        //IntakeSubsystem intake = new IntakeSubsystem(0, 0, 1, 2, 3, "rio");
//  public IntakeSubsystem(int lowerIntakeId, int spinIntakeId, int channel1, int channel2, int channel3, String can)  {

    public ArmContainer() {
        configureBindings();

    }


    private void configureBindings() {
    //    m_driverController.a().onTrue(new SwingForward(arm, 270, 10, 10, .25));

    //.01 velocity for 1st time testing
    //m_driverController.a().onTrue(new SwingForward(arm, 10, .5, 1, .25).andThen(new SwingBack(arm, .5, 1, .25)));
    double velocity = .03;
         m_driverController.a().onTrue(new SwingForward(arm, -90, velocity, 2, .25));
         m_driverController.b().onTrue(new SwingForward(arm, 90, velocity, 2, .25));

        // m_driverController.b().onTrue(new SwingForward(arm, 0, velocity, 2, .25));
        // m_driverController.y().onTrue(new SwingForward(arm, 90, velocity, 2, .25));
        // m_driverController.x().onTrue(new SwingForward(arm, -90, velocity, 2, .25));


          m_driverController.x().onTrue(new SwingForwardServo(arm).andThen(Commands.waitSeconds(1.0)).andThen(new SwingBackServo(arm)));
        //  m_driverController.a().onTrue(new LowerArmIntake(intake, 270).andThen(new IntakePickUpFromGround(intake)).andThen(new UpperArmIntake(intake)).andThen(new IntakePassNoteToPizzaBox(intake)));
        //  m_driverController.y().onTrue(new SpinNoteContainerMotor(arm, .25, 10).alongWith(new SwingForward(arm, 180, 5, 5, .25)));

        //IMPORTANT m_driverController.y().onTrue(new SpinAndSwing(arm));
        //command that loads the note
        // m_driverController.leftBumper().onTrue(new LowerArmIntake().andThen(new StartIntake()).andThen(new SwingForward()).andThen(new UpperIntake()));

    //    SpinNoteContainerMotor army = new SpinNoteContainerMotor (arm, 0.25, 10);
    //    m_driverController.y().onTrue(army);
       //m_driverController.x().onTrue(new StopNoteContainerMotor(arm));
       

//We might not need this anymore (2/10/24)
        Shuffleboard.getTab("Arm").addDouble("encoder",()->arm.encoderGetAngle());
        Shuffleboard.getTab("Arm").addDouble("Arm Value",()->arm.getArmAngle());

        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
