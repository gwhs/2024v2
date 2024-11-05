// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTagCam;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


/** Add your docs here. */
public class AprilTagCam {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonCamera cam;
    SwerveDrivePoseEstimator m_estimator;
    PhotonPoseEstimator estim; 

    // 
    public AprilTagCam(String str, SwerveDrivePoseEstimator estimator ){
        cam = new PhotonCamera(str);
        m_estimator = estimator; 
        estim = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, null);
        
    }   

    public void updatePoseEstim(){

        System.out.println("Spillard  hajel ");

        // write an if statement that allows to find if the the list is empty or not
        // getting the unread results target pose 
        // we need to get the robot pose from the target pose 
        // using the update method, in photonPoseEstimator, get an estimated robot pose
        // using this we can get pose3D and turn it into pose2d
        // we need to give the info of where the robot is to the drive train so it knows where to move 

        var results = cam.getAllUnreadResults(); 
        for( PhotonPipelineResult targetPose : results ){
            
            System.out.println(targetPose);

            

            Optional<EstimatedRobotPose> optionalEstimPose = (estim.update(targetPose)); 
            Pose3d estimPose3d ; 

            if(!optionalEstimPose.isEmpty()){
                estimPose3d = optionalEstimPose.get().estimatedPose;
            }
            else{
                return; 
            }
            Pose2d pos = estimPose3d.toPose2d(); // yay :0 im so happy
            double timestamp = targetPose.getTimestampSeconds();
            
            
        }   
    
       
    }

}
