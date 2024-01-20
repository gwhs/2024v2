// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//* Calculates error distance from LimeLight Tx */

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimeVision.LimeLightSub;

public class PIDMove extends SubsystemBase {

private PIDController controller;
private LimeLightSub limeLightSub;

  public PIDMove(double kP, double Ki, double kD, double setPoint) {
    controller = new PIDController(kP, Ki, kD);
    limeLightSub = new LimeLightSub("limelight");

    controller.setSetpoint(setPoint);

  }

  public double getError() {
    return controller.calculate(limeLightSub.getTx());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run duriney simulation
  }
}
