// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeIOSim implements IntakeIO {
  private SingleJointedArmSim intakeArmSim = new SingleJointedArmSim(
      DCMotor.getFalcon500Foc(1),
      100,
      SingleJointedArmSim.estimateMOI(.15, 5),
      .15,
      Units.degreesToRadians(0),
      Units.degreesToRadians(95),
      true,
      Units.degreesToRadians(IntakeConstants.UP_POSITION));

  private FlywheelSim intakeSpinSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.0001, 1),
      DCMotor.getKrakenX60Foc(1));
  private boolean noteSensor = false;

  public IntakeIOSim() {
    SmartDashboard.putData("Simulation/Note Sensor", Commands.runOnce(() -> noteSensor = !noteSensor));
  }

  public double getIntakeArmAngle() {
    return Units.radiansToDegrees(intakeArmSim.getAngleRads());
  }

  public void setArmSpeed(double speed) {
    intakeArmSim.setInputVoltage(speed * 10);
  }

  public void setSpinSpeed(double speed) {
    intakeSpinSim.setInputVoltage(speed * 10);
  }

  public boolean getNoteSensor() {
    return noteSensor;
  }

  public double getSpinSpeed() {
    return Units.radiansToRotations(intakeSpinSim.getAngularVelocityRadPerSec());
  }

  public void update() {
    intakeArmSim.update(0.02);
    intakeSpinSim.update(0.02);
  }

}
