// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class UtilMath {
    public static final double BLUE_SPEAKER_X = -0.0381; //in meters
    public static final double BLUE_SPEAKER_Y = 5.5479; //in meters
    public static final double RED_SPEAKER_X = 16.5793; //in meters
    public static final double RED_SPEAKER_Y = 5.5479; //in meters
    


    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        double calucatedRad = Math.atan((targetY-pose.getY())/ (targetX-pose.getX()));
        
        return  Math.toDegrees(calucatedRad);
    }

    public static double BLUESpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
    }

    public static double REDSpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, RED_SPEAKER_X, RED_SPEAKER_Y);
    
    }
}
