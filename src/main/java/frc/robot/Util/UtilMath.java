// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class UtilMath {
    public static final double BLUE_SPEAKER_X = -0.0381; //in meters
    public static final double BLUE_SPEAKER_Y = 5.5479; //in meters
    public static final double RED_SPEAKER_X = 16.5793; //in meters
    public static final double RED_SPEAKER_Y = 5.5479; //in meters
    
    public static final int RED_SOURCE_HEADING_ANGLE = -120;
    public static final int BLUE_SOURCE_HEADING_ANGLE = -60;



    public static double caclucateRotateTheta(Pose2d pose, double targetX, double targetY){
        double calucatedRad = Math.atan((targetY-pose.getY())/ (targetX-pose.getX()));
        
        return  Math.abs(Math.toDegrees(calucatedRad));
    }

    public static double SourceIntakeHeading(Pose2d pose)
    {
        if(DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
            return BLUE_SOURCE_HEADING_ANGLE;
        }
        else if(DriverStation.getAlliance().isPresent() &&
         DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            return RED_SOURCE_HEADING_ANGLE;
        }
        else
        {
            return pose.getRotation().getDegrees();
        }
    }

    public static double FrontSpeakerTheta(Pose2d pose)
    {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
             if(pose.getY() >= BLUE_SPEAKER_Y)
        {
            return -180+caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
        }
        else if(pose.getY() <= BLUE_SPEAKER_Y)
        {
            return 180-caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
        }
        }
        else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            if(pose.getY() >= RED_SPEAKER_Y)
            {
                return -caclucateRotateTheta(pose, RED_SPEAKER_X, RED_SPEAKER_Y);
            }
            else if(pose.getY() <= RED_SPEAKER_Y)
            {
                return caclucateRotateTheta(pose, RED_SPEAKER_X, RED_SPEAKER_Y);
            }
        }
        return caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
       
    }
     public static double BackSpeakerTheta(Pose2d pose)
    {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
             if(pose.getY() >= BLUE_SPEAKER_Y)
        {
            return caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
        }
        else if(pose.getY() <= BLUE_SPEAKER_Y)
        {
            return -caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
        }
        }
        else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            if(pose.getY() >= RED_SPEAKER_Y)
            {
                return 180-caclucateRotateTheta(pose, RED_SPEAKER_X, RED_SPEAKER_Y);
            }
            else if(pose.getY() <= RED_SPEAKER_Y)
            {
                return -180+caclucateRotateTheta(pose, RED_SPEAKER_X, RED_SPEAKER_Y);
            }
        }
        return caclucateRotateTheta(pose, BLUE_SPEAKER_X, BLUE_SPEAKER_Y);
       
    }

    public static double distanceFromSpeaker(Pose2d pose) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return Math.sqrt(Math.pow(BLUE_SPEAKER_X - robotX, 2) + Math.pow(BLUE_SPEAKER_Y - robotY, 2));
        } else {
            return Math.sqrt(Math.pow(RED_SPEAKER_X - robotX, 2) + Math.pow(RED_SPEAKER_Y - robotY, 2));
        }
    }
}
