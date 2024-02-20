// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */

public class UtilMath {
    public static final double BLUE_SPEAKER_X = -0.0381; //in meters
    public static final double BLUE_SPEAKER_Y = 5.5479; //in meters
    public static final double RED_SPEAKER_X = 16.5793; //in meters
    public static final double RED_SPEAKER_Y = 5.5479; //in meters
    


    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        double calucatedRad = Math.atan2((targetY-pose.getY()), (targetX-pose.getX()));
        return  Math.toDegrees(calucatedRad);
    }

    public static double BLUESpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, BLUE_SPEAKER_Y, BLUE_SPEAKER_X);
    }

    public static double REDSpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, RED_SPEAKER_Y, RED_SPEAKER_X);
    
    }
    public static double inchesToMeters(double inches)
    {
        double meters = inches/39.97;
        return meters;
    }
    public static int whichAmp(Pose2d pose){
        if(pose.getX() < 324.865){
            return 6;
        }
        else if (pose.getX() > 324.865){
            return 5;
    }
    return 0;
    }
}
