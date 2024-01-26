package frc.robot.testcontainers;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimeVision.PIDMove;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import java.io.File;




public class VisionContainer implements BaseContainer {


    private PIDMove PIDMove;
    
    // todo: add intake subsystem
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private LimeLightSub limeLightSub = new LimeLightSub("limelight");

    public String getDriveTrainName(){
        return "swerve/vision";
      }
    
    public VisionContainer() {
        configureBindings();
    }


    private void configureBindings() {
        
    }
}
