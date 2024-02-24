// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.security.CodeSigner;
import java.sql.Driver;

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
          if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
             SPEAKER_X = Constants.FieldConstants.BLUE_SPEAKER_X;
             SPEAKER_Y = Constants.FieldConstants.BLUE_SPEAKER_Y;
        }
        else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
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

    public static double distanceFormula(double x1, double y1, double x2, double y2)
    {   
        double dist = Math.sqrt( ( ( (x2 - x1)*(x2-x1) ) + ( (y2 - y1)*(y2 - y1) ) ));
        return dist;
    }

    public static int bestTrapID(Pose2d pose)
    {
        double robotX = pose.getX();
        double robotY = pose.getY();

        double distanceToTrap15 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.BLUE_TRAP_15_X, Constants.FieldConstants.BLUE_TRAP_15_Y);
        double distanceToTrap16 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.BLUE_TRAP_16_X, Constants.FieldConstants.BLUE_TRAP_16_Y);
        double distanceToTrap14 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.BLUE_TRAP_14_X, Constants.FieldConstants.BLUE_TRAP_14_Y);

        double distanceToTrap11 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.RED_TRAP_11_X, Constants.FieldConstants.RED_TRAP_11_Y);
        double distanceToTrap12 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.RED_TRAP_12_X, Constants.FieldConstants.RED_TRAP_12_Y);
        double distanceToTrap13 = distanceFormula
        (robotX, robotY, Constants.FieldConstants.RED_TRAP_13_X, Constants.FieldConstants.RED_TRAP_13_Y);

    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
    {
        if(distanceToTrap15 < distanceToTrap16 && distanceToTrap15 < distanceToTrap14)
        {
            return 15;
        }
        else if(distanceToTrap16 < distanceToTrap15 && distanceToTrap16 < distanceToTrap14)
        {
            return 16;
        }
        else if(distanceToTrap14 < distanceToTrap16 && distanceToTrap14 < distanceToTrap15)
        {
            return 14;
        }
    }
    else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {

        if(distanceToTrap11 < distanceToTrap12 && distanceToTrap11 < distanceToTrap13)
        {
            return 11;
        }
        else if(distanceToTrap12 < distanceToTrap11 && distanceToTrap11 < distanceToTrap13)
        {
            return 12;
        }
        else if(distanceToTrap13 < distanceToTrap12 && distanceToTrap13 < distanceToTrap11)
        {
            return 13;
        }

    }

    return 0;


}
}
