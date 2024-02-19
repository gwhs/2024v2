// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class UtilMath {

    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        double calucatedRad = Math.atan2((targetY-pose.getY()), (targetX-pose.getX()));
        return  Math.toDegrees(calucatedRad);
    }

     public static double BLUESpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, Constants.FieldConstants.BLUE_SPEAKER_Y, Constants.FieldConstants.BLUE_SPEAKER_X);
    }

    public static double REDSpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, Constants.FieldConstants.RED_SPEAKER_Y, Constants.FieldConstants.RED_SPEAKER_X);
    
    }

    //Sims don't work for this method
    public static double SPEAKER_X;
    public static double SPEAKER_Y;

    public static double SpeakerTheta(Pose2d pose)
    {
          if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
             SPEAKER_X = Constants.FieldConstants.BLUE_SPEAKER_X;
             SPEAKER_Y = Constants.FieldConstants.BLUE_SPEAKER_Y;
        }
        else
        {
            SPEAKER_X = Constants.FieldConstants.RED_SPEAKER_X;
            SPEAKER_Y = Constants.FieldConstants.RED_SPEAKER_Y;
        }

        return caclucateRotateTheta(pose, SPEAKER_X, SPEAKER_Y);
    }
    
public static double inchesToMeters(double inches)
{
  double meters = inches/39.97;
  return meters;
}

public static int bestTrap(Pose2d pose)
{
  if(Math.abs(pose.getY() - Constants.FieldConstants.BLUE_TRAP_16_Y) < Math.abs(pose.getY() - Constants.FieldConstants.BLUE_TRAP_15_Y))
  {
    return 16; //returns id
  }
  else if(Math.abs(pose.getY() - Constants.FieldConstants.BLUE_TRAP_14_Y) < (Math.abs(pose.getY() - Constants.FieldConstants.BLUE_TRAP_15_Y)))
  {
    return 14;
  }
  else
  {
    return 15;
  }
}
}
