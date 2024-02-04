package frc.robot.testcontainers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.SpinNoteContainerMotor;
import frc.robot.commands.StopNoteContainerMotor;
import frc.robot.commands.SwingBack;
import frc.robot.commands.SwingForward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class ArmContainer implements BaseContainer {
  
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
        ArmSubsystem arm = new ArmSubsystem(8, "rio", 0, "rio", 0, 1); 

    public ArmContainer() {
        configureBindings();

    }


    private void configureBindings() {
       m_driverController.a().onTrue(new SwingForward(arm, 180, 3, 10, .25));
       m_driverController.b().onTrue(new SwingBack(arm, -3, 10, .25));

       //SpinNoteContainerMotor army = new SpinNoteContainerMotor (arm, 0.25, 10);
       //m_driverController.y().onTrue(army);
       //m_driverController.x().onTrue(new StopNoteContainerMotor(arm));
       


        Shuffleboard.getTab("aaaaaa").addDouble("encoder",()->arm.encoderGetAngle());
        Shuffleboard.getTab("aaaaaa").addDouble("arm",()->arm.getArmAngle());

        


        // DigitalInput sensor = new DigitalInput(3);
        // Shuffleboard.getTab("aaaaaa").addBoolean("sensor",()->sensor.get());
    }

}
