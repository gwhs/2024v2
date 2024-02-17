package frc.robot.testcontainers;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimberCommands.MotorUp;
import frc.robot.commands.ClimberCommands.ClimbDown;
import frc.robot.commands.ClimberCommands.ClimbUp;
import frc.robot.commands.ClimberCommands.MotorDown;
import frc.robot.commands.ledcommands.ChangeLEDToGreen;
import frc.robot.commands.ledcommands.ChangeLEDToRed;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbContainer implements BaseContainer {

    Climbsubsystem climbsubsystem = new Climbsubsystem( ClimbConstants.MOTOR_LEFT_ID, 
                                                        ClimbConstants.MOTOR_RIGHT_ID, 
                                                        ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                        ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio"); //change arguments

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                        getDriveTrainName()));
  
    // NEED TO DO: climb command should autoalign then climb then shoot and then maybe climb back down
    private final CommandXboxController m_driverController =
        new CommandXboxController(0);

    


    public ClimbContainer() {
        Shuffleboard.getTab("Climb").addDouble("climb distance left", () -> climbsubsystem.getPositionLeft());
        Shuffleboard.getTab("Climb").addDouble("climb distance right", () -> climbsubsystem.getPositionRight());
        
        configureBindings();
    
    }


    private void configureBindings() {
        m_driverController.a().whileTrue(new MotorUp(climbsubsystem, swerve));
        m_driverController.b().whileTrue(new MotorDown(climbsubsystem, swerve));
        //m_driverController.x().onTrue(new Trap());

        Shuffleboard.getTab("Climb").add("motor down", new MotorDown(climbsubsystem, swerve));
        Shuffleboard.getTab("Climb").add("motor up", new MotorUp(climbsubsystem, swerve));
        Shuffleboard.getTab("Climb").add("climb up", new ClimbUp(climbsubsystem, swerve));
        Shuffleboard.getTab("Climb").add("climb down", new ClimbDown(climbsubsystem, swerve));
                
    }

    public String getDriveTrainName(){
        return "swerve/ryker_falcon";
      }
}
