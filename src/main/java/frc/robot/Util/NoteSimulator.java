// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class NoteSimulator {
  private static Pose3d currentNotePose = new Pose3d();
  private static Translation3d noteVelocity = new Translation3d();
  private static boolean noteInPB = false;
  private static List<Translation3d> noteTrajectory = new ArrayList<>();

  private static final double AIR_DENSITY = 1.225;
  private static final double DRAG_COEFFICIENT = 0.45;
  private static final double CROSSECTION_AREA = 0.11;
  private static final double MASS = 0.235;

  public static final double MAX_VELOCITY = 10;

  public static final Transform3d robotToPivot = new Transform3d(0, 0, 0.78, new Rotation3d());

  private static final StructPublisher<Pose3d> currentNotePosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Note Simulator/Current Note", Pose3d.struct).publish();

  private static final StructArrayPublisher<Translation3d> noteTrajectoryPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Note Simulator/Note Trajectory", Translation3d.struct).publish();

  public static void update(Pose2d robotPose, double armAngle) {
    if(noteInPB) {
      currentNotePose = getNoteInPizzaboxPose(robotPose, armAngle);
      currentNotePosePublisher.accept(currentNotePose);
    }
    else {
      double dt = 0.02;
      Translation3d posDelta = noteVelocity.times(dt);

      currentNotePose = new Pose3d(currentNotePose.getTranslation().plus(posDelta), currentNotePose.getRotation());

      currentNotePosePublisher.accept(currentNotePose);

      if (currentNotePose.getX() <= -0.25
          || currentNotePose.getX() >= 16.54 + 0.25
          || currentNotePose.getY() <= -0.25
          || currentNotePose.getY() >= 8.21 + 0.25
          || currentNotePose.getZ() <= 0.0) {
        noteVelocity = new Translation3d();
      } 
      else {
        noteVelocity = noteVelocity.minus(new Translation3d(0.0, 0.0, 9.81 * dt));
        double norm = noteVelocity.getNorm();

        double fDrag = 0.5 * AIR_DENSITY * Math.pow(norm, 2) * DRAG_COEFFICIENT * CROSSECTION_AREA;
        double deltaV = (MASS * fDrag) * dt;

        double t = (norm - deltaV) / norm;
        noteVelocity = noteVelocity.times(t);
        noteTrajectory.add(currentNotePose.getTranslation());

        noteTrajectoryPublisher.accept(noteTrajectory.toArray(new Translation3d[noteTrajectory.size()]));
      }
    }
  }

  public static Pose3d getNoteInPizzaboxPose(Pose2d robotPose, double armAngle) {
    Transform3d pivotToArm = new Transform3d(
        .38 * -Math.cos(Units.degreesToRadians(armAngle)), 
        0, 
        .38 * -Math.cos(Units.degreesToRadians(armAngle - 90)), 
        new Rotation3d(
          0, 
          Units.degreesToRadians(90 - armAngle), 
          0));

    Transform3d armToNote = new Transform3d(0, 0, -0.04, new Rotation3d());

    return new Pose3d(robotPose).transformBy(robotToPivot).transformBy(pivotToArm).transformBy(armToNote);
  }

  public static Command intakeNote() {
    return Commands.runOnce(() -> {
      noteInPB = true;
      noteTrajectory.clear();
    });
  }

  public static Command launchNote(double velocity, ChassisSpeeds robotVelocity, Pose2d robotPose, double armAngle) {
    return Commands.runOnce(() -> {
      if (!noteInPB) {
        return;
      }

      currentNotePose = getNoteInPizzaboxPose(robotPose, armAngle);
      noteVelocity = new Translation3d(velocity, currentNotePose.getRotation());

      ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, robotPose.getRotation());
      noteVelocity = noteVelocity.plus(new Translation3d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond, 0.0));

      noteInPB = false;
    });
  }
}
