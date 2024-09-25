package frc.robot.subsystems.Arm;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, ArmConstants.ARM_MOTOR_CAN);
  private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.ARM_ENCODER_CHANNEL);
  private Constraints constraints = new Constraints(ArmConstants.ARM_VEL, ArmConstants.ARM_ACC);
  private ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.ARM_kP, ArmConstants.ARM_kI,
      ArmConstants.ARM_kD, constraints);

  public ArmSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout armCommandsLayout = tab.getLayout("Arm Commands", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN"));

    armCommandsLayout.add(spinArm(120).withName("spinArm120"));
    armCommandsLayout.add(spinArm(60).withName(("spinArm60")));
    pidController.setGoal(Units.degreesToRadians(90));
    armCommandsLayout.add(spinArm(200).withName("spinArm200"));
  }

  public double getArmEncoderAngle() {
    return Units.rotationsToDegrees(m_armEncoder.getAbsolutePosition()) - ArmConstants.ARM_ENCODER_OFFSET;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pidController.calculate(Units.degreesToRadians(getArmEncoderAngle()));

    pidOutput = MathUtil.clamp(pidOutput, -1, 1);
    m_armMotor.set(-pidOutput);
    NetworkTableInstance.getDefault().getEntry("Arm/ArmAngle").setNumber(getArmEncoderAngle());
    NetworkTableInstance.getDefault().getEntry("Arm/pidOutput").setNumber(pidOutput);
    NetworkTableInstance.getDefault().getEntry("Arm/armGoal")
        .setNumber(Units.radiansToDegrees(pidController.getGoal().position));
  }

  public Command spinArm(double targetAngle) {
    return this.runOnce(() -> {
      pidController.setGoal(Units.degreesToRadians(targetAngle));
    }).andThen(Commands.waitUntil(() -> pidController.atGoal()));
  }
}
