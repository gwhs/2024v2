package frc.robot.subsystems.Climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {

  private DCMotorSim rightMotorSim = new DCMotorSim(
      DCMotor.getFalcon500Foc(1),
      1, 0.0001);


  private DCMotorSim leftMotorSim = new DCMotorSim(
      DCMotor.getFalcon500Foc(1),
      1, 0.0001);


  @Override
  public double getRightMotorPosition() {
    return rightMotorSim.getAngularPositionRotations();
  }


  @Override
  public double getLeftMotorPosition() {
    return leftMotorSim.getAngularPositionRotations();
  }


  @Override
  public void setLeftMotorSpeed(double speed) {
    leftMotorSim.setInputVoltage(speed*10);
  }
 



  @Override
  public void setRightMotorSpeed(double speed) {
    rightMotorSim.setInputVoltage(speed*10);
  }
  public void update() {
    rightMotorSim.update(.020);
    leftMotorSim.update(.020);
  }


  @Override
  public boolean getTopLeftLimitSwitch() {
    return getLeftMotorPosition() <= ClimbConstants.LEFT_UP_POSITION;
  }


  @Override
  public boolean getTopRightLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTopRightLimitSwitch'");
  }


  @Override
  public boolean getBottomLeftLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBottomLeftLimitSwitch'");
  }


  @Override
  public boolean getBottomRightLimitSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBottomRightLimitSwitch'");
  }
}
