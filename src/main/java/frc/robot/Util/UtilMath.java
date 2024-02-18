// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public class UtilMath {
    public static double SPEAKER_X;
    public static double SPEAKER_Y;
    


    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        double calucatedRad = Math.atan2((targetY-pose.getY()), (targetX-pose.getX()));
        return  Math.toDegrees(calucatedRad);
    }
//Sims don't work for this method
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



}
