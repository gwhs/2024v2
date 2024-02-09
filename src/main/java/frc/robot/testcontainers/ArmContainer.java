package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.Arm.SpinNoteContainerMotor;
import frc.robot.commands.Arm.StopNoteContainerMotor;
import frc.robot.commands.Arm.SwingBack;
import frc.robot.commands.Arm.SwingForward;
import frc.robot.commands.Arm.SwingServo;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class ArmContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        ArmSubsystem arm = new ArmSubsystem(8, "rio", 0, "rio", 0, 1, 0); 

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
       


        Shuffleboard.getTab("Arm").addDouble("encoder",()->arm.encoderGetAngle());
        Shuffleboard.getTab("Arm").addDouble("arm",()->arm.getArmAngle());

        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
