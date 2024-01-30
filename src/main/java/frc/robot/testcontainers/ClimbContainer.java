package frc.robot.testcontainers;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimberCommands.Climb;
import frc.robot.commands.ClimberCommands.ClimbDown;
import frc.robot.commands.ClimberCommands.ClimbUp;
import frc.robot.commands.ledcommands.ChangeLEDToGreen;
import frc.robot.commands.ledcommands.ChangeLEDToRed;
import frc.robot.subsystems.Climbsubsystem;

public class ClimbContainer implements BaseContainer {

    Climbsubsystem climbsubsystem = new Climbsubsystem(ClimbConstants.MOTOR_LEFT_ID, ClimbConstants.MOTOR_RIGHT_ID, ClimbConstants.MOTOR_LEFT_INVERTED, ClimbConstants.MOTOR_RIGHT_INVERTED, "rio"); //change arguments
  
    // NEED TO DO: climb command should autoalign then climb then shoot and then maybe climb back down
    private final CommandXboxController m_driverController =
        new CommandXboxController(0);

    


    public ClimbContainer() {
        
        configureBindings();
    
    }


    private void configureBindings() {
        m_driverController.a().whileTrue(new ClimbDown(climbsubsystem));
        m_driverController.b().whileTrue(new ClimbUp(climbsubsystem));
        //m_driverController.x().onTrue(new Trap());

        Shuffleboard.getTab("Climb").add("climb up", new ClimbUp(climbsubsystem));
        Shuffleboard.getTab("Climb").add("climb down", new ClimbDown(climbsubsystem));
                
    }
}
