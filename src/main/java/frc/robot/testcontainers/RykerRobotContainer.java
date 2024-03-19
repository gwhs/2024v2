package frc.robot.testcontainers;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.driveCommands.*;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class RykerRobotContainer implements BaseContainer {

  CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SwerveSubsystem m_drivebase;

  private final TeleopDrive closedFieldRel;

  public String getDriveTrainName(){
    return "swerve/ryker_falcon";
  }


  public RykerRobotContainer() {
      m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), getDriveTrainName()));

      closedFieldRel = new TeleopDrive(
                                        m_drivebase,
                                        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
                                        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
                                        () -> driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis(), () -> true);

      m_drivebase.setDefaultCommand(closedFieldRel);

      configureBindings();
  }

  private void configureBindings() {
    driverController.x().whileTrue(new DecreaseSpeed(closedFieldRel));

    driverController.start().onTrue(new InstantCommand(m_drivebase::zeroGyro));
  }
}