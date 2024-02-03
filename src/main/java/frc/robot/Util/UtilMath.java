// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class UtilMath {
    public static final double SPEAKER_X = 5.5479; //in meters
    public static final double SPEAKER_Y = -0.0254; //in meters

    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        Math.atan((pose.getX()+targetX)/(targetY-pose.getY()));
        return  Math.atan((pose.getX()+targetX)/(targetY-pose.getY()));
    }

    public static double SpeakerTheta(Pose2d pose)
    {

        return caclucateRotateTheta(pose, SPEAKER_Y, SPEAKER_X);
    }
}
