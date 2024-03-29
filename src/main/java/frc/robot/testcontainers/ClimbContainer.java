package frc.robot.testcontainers;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Climbsubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ClimbContainer implements BaseContainer {

    CommandXboxController driverXbox = new CommandXboxController(0);

    Climbsubsystem climbsubsystem = new Climbsubsystem( ClimbConstants.MOTOR_LEFT_ID, 
                                                        ClimbConstants.MOTOR_RIGHT_ID, 
                                                        ClimbConstants.MOTOR_LEFT_INVERTED, 
                                                        ClimbConstants.MOTOR_RIGHT_INVERTED, 
                                                        "rio"); //change arguments

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                        getDriveTrainName()));

    TeleopDrive closedFieldRel = new TeleopDrive(
        swerve,
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis(), () -> true);
  
    // NEED TO DO: climb command should autoalign then climb then shoot and then maybe climb back down
    // private final CommandXboxController m_driverController =
    //     new CommandXboxController(0);

    


    public ClimbContainer() {
        Shuffleboard.getTab("Climb").addDouble("climb distance left", () -> climbsubsystem.getPositionLeft());
        Shuffleboard.getTab("Climb").addDouble("climb distance right", () -> climbsubsystem.getPositionRight());
        Shuffleboard.getTab("Climb").addBoolean("bot left limit", () -> climbsubsystem.getBotLeftLimit());
        Shuffleboard.getTab("Climb").addBoolean("bot right limit", () -> climbsubsystem.getBotRightLimit());
        Shuffleboard.getTab("Climb").addBoolean("top left limit", () -> climbsubsystem.getTopLeftLimit());
        Shuffleboard.getTab("Climb").addBoolean("top right limit", () -> climbsubsystem.getTopRightLimit());

        // Shuffleboard.getTab("Climb").addDouble("roll", () -> swerve.getRoll().getDegrees());
        // Shuffleboard.getTab("Climb").addDouble("pitch", () -> swerve.getYaw().getDegrees());
        // Shuffleboard.getTab("Climb").addDouble("yaw", () -> swerve.getPitch().getDegrees());
        
        swerve.setDefaultCommand(closedFieldRel);  //TO CHANGE DRIVE BASE


        configureBindings();
    
    }


    private void configureBindings() {
                
    }

    public String getDriveTrainName(){
        return "swerve/hajel_kraken";
      }
}
