// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.LimeLight;

// import frc.robot.subsystems.LimeVision.LimeLightSub;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;

// /** An example command that uses an example subsystem. */
// public class SmoothAlign extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final SwerveSubsystem driSwerveSubsystem;
//   private final LimeLightSub limeLightSub;

//   private double distanceX;
//   private double distanceY;
//   private double rotationTheta;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public SmoothAlign(SwerveSubsystem driSwerveSubsystem, LimeLightSub limeLightSub) {
//     this.driSwerveSubsystem = driSwerveSubsystem;
//     this.limeLightSub = limeLightSub;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driSwerveSubsystem, limeLightSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     limeLightSub.setPoint(2,"x");
//     limeLightSub.setPoint(0, "y");
//     limeLightSub.setPoint(0, "theta");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     distanceX = limeLightSub.getErrorX();
//     distanceY = limeLightSub.getErrorY();
//     rotationTheta = limeLightSub.getSmoothThetaError();
//     driSwerveSubsystem.drive(new Translation2d(distanceX, distanceY), rotationTheta, true);
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (limeLightSub.getErrorX() < 0.1 && limeLightSub.getErrorX() > -0.01); // when robot is infront of the tag
//   }
// }
