// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
//not using atm
public class SpinArmPID extends PIDCommand {
    private double angle;
    static final PIDController armController = new PIDController(.005, .005, .0);


  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  public SpinArmPID(ArmSubsystem armSubsystem, final double targetAngle) {
    super(armController, ()-> armSubsystem.encoderGetAngle(), () -> targetAngle,
            (final double speed) -> 
            {armSubsystem.spinArm(-speed);
            System.out.println(speed);}
            , armSubsystem);
    angle = targetAngle;
    this.armSubsystem = armSubsystem;

  }
//
//  Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     double encoderAng = armSubsystem.encoderGetAngle();
//     return Math.abs(angle - encoderAng) < 1; 
//   }
}
