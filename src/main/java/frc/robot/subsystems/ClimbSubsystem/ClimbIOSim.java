package frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
<<<<<<< HEAD
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
=======
import edu.wpi.first.math.system.plant.LinearSystemId;
>>>>>>> 2025-beta
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {

  private DCMotorSim rightMotorSim = new DCMotorSim(
<<<<<<< HEAD
      DCMotor.getFalcon500Foc(1),
      1, 0.001);

  private DCMotorSim leftMotorSim = new DCMotorSim(
      DCMotor.getFalcon500Foc(1),
      1, 0.001);
  private Constraints constraints = new Constraints(ClimbConstants.MAX_VELOCITY, ClimbConstants.MAX_ACCELERATION);
  private ProfiledPIDController leftpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP,
      ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);
  private ProfiledPIDController rightpidController = new ProfiledPIDController(ClimbConstants.CLIMB_PID_KP,
      ClimbConstants.CLIMB_PID_KI, ClimbConstants.CLIMB_PID_KD, constraints);

  @Override
  public void setPositionLeft(double position) {
      leftpidController.setGoal(position);
  }

  public void setPositionRight(double position) {
      rightpidController.setGoal(position);
  }
=======
      LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1),
      DCMotor.getFalcon500(1));

  private DCMotorSim leftMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.0001, 1),
      DCMotor.getFalcon500(1));
>>>>>>> 2025-beta

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
    leftMotorSim.setInputVoltage(speed * 10);
  }

  @Override
  public void setRightMotorSpeed(double speed) {
    rightMotorSim.setInputVoltage(speed * 10);
  }

  public void update() {
    rightMotorSim.update(.020);
    leftMotorSim.update(.020);

    double pidOutputLeft = leftpidController.calculate(getLeftMotorPosition());
    double pidOutputRight = rightpidController.calculate(getRightMotorPosition());

    setLeftMotorSpeed(pidOutputLeft);
    setRightMotorSpeed(pidOutputRight);

  }
}
