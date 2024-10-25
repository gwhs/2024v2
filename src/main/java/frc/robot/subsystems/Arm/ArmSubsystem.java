package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO armIO;
  private Constraints constraints = new Constraints(ArmConstants.ARM_VEL, ArmConstants.ARM_ACC);
  private ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.ARM_kP, ArmConstants.ARM_kI,
      ArmConstants.ARM_kD, constraints);

  public ArmSubsystem() {
    if (RobotBase.isSimulation()) {
      armIO = new ArmIOSim();
      NetworkTableInstance.getDefault().getEntry("/Arm/Mode").setString("Simulation");
    }
    else {
      armIO = new ArmIOReal();
      NetworkTableInstance.getDefault().getEntry("/Arm/Mode").setString("Real");
    }

    pidController.setTolerance(Units.degreesToRadians(3));

    SmartDashboard.putData(spinArm(120).withName("spinArm120"));
    SmartDashboard.putData(spinArm(60).withName(("spinArm60")));
    pidController.setGoal(Units.degreesToRadians(90));
    SmartDashboard.putData(spinArm(200).withName("spinArm200"));
  }

  public double getArmAngle() {
    return armIO.getArmEncoderAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pidController.calculate(Units.degreesToRadians(armIO.getArmEncoderAngle()));

    pidOutput = MathUtil.clamp(pidOutput, -1, 1);
    armIO.setArmSpeed(pidOutput);
    NetworkTableInstance.getDefault().getEntry("/Arm/ArmAngle").setNumber(armIO.getArmEncoderAngle());
    NetworkTableInstance.getDefault().getEntry("/Arm/pidOutput").setNumber(pidOutput);
    NetworkTableInstance.getDefault().getEntry("/Arm/armGoal")
        .setNumber(Units.radiansToDegrees(pidController.getGoal().position));
    armIO.update();
  }

  public Command spinArm(double targetAngle) {
    return this.runOnce(() -> {
      pidController.setGoal(Units.degreesToRadians(targetAngle));
    }).andThen(Commands.waitUntil(() -> pidController.atGoal()));
  }
}