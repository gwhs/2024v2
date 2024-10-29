// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTagCam;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


/** Add your docs here. */
public class AprilTagCam {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonCamera cam;
    SwerveDrivePoseEstimator m_estimator;

    // 
    public AprilTagCam(String str, SwerveDrivePoseEstimator estimator ){
        cam = new PhotonCamera(str);
        m_estimator = estimator; 
        
    }   

    public void updatePoseEstim(){

        System.out.println("Spillard  hajel ");

        // write an if statement that allows to find if the the list is empty or not
        // getting the unread results target pose 
        // we need to get the robot pose from the target pose 
        //
        // we need to give the info of where the robot is to the drive train so it knows where to move 

        var results = cam.getAllUnreadResults(); 
        for( PhotonPipelineResult targetPose : results ){
            System.out.println(targetPose);
            // 2dPose robotPose = new 2dPose();

        }   
    
       
    }

}
