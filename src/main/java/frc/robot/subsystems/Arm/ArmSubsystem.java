package frc.robot.subsystems.Arm;


import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;


public class ArmSubsystem extends SubsystemBase {
  private TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, ArmConstants.ARM_MOTOR_CAN);
 private DutyCycleEncoder m_armEncoder = m_armEncoder = new DutyCycleEncoder(ArmConstants.ARM_ENCODER_CHANNEL);
 private Constraints constraints = new Constraints(ArmConstants.ARM_VEL, ArmConstants.ARM_ACC);
 private ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.ARM_kP, ArmConstants.ARM_kI, ArmConstants.ARM_kP, constraints);


 public ArmSubsystem() {
 }
  public double getArmEncoderAngle() {
   return Units.rotationsToDegrees(m_armEncoder.getAbsolutePosition()) - ArmConstants.ARM_ENCODER_OFFSET;


 }


 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   double pidOutput = pidController.calculate(Units.degreesToRadians(getArmEncoderAngle()));


   pidOutput = MathUtil.clamp(pidOutput, -1, 1);
   m_armMotor.set(pidOutput);
 }


 public Command spinArm(double targetAngle) {
   return this.runOnce(() -> {
     pidController.setGoal(Units.degreesToRadians(targetAngle));
   }).andThen(Commands.waitUntil(() -> pidController.atGoal()));
 }
}

