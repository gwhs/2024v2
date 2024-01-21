// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//* Calculates error distance from LimeLight Tx */

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimeVision.LimeLightSub;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.parser.SwerveDriveConfiguration;


public class PIDMove extends SubsystemBase {

private PIDController controller;
private SwerveSubsystem swerveSubsystem;
private LimeLightSub limeLightSub;

// private LimeLightSub limeLightSub = new LimeLightSub("limelight");

  public PIDMove(SwerveSubsystem swerveSystem, LimeLightSub limeLightSub, double kP, double Ki, double kD, double setPoint) {
    this.limeLightSub = limeLightSub;
    this.swerveSubsystem = swerveSystem;
    
    controller = new PIDController(kP, Ki, kD);
    controller.setSetpoint(setPoint);

  }

  public double getError(LimeLightSub limeLightSub) {
    return controller.calculate(limeLightSub.getTx());
  }

}
