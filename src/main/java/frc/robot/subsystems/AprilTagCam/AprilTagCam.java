// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTagCam;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


/** Add your docs here. */
public class AprilTagCam {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonCamera cam;
    PhotonPoseEstimator estim;
    Consumer<AprilTagHelp> addVisionMeasurement;

    // 
    public AprilTagCam(String str, Consumer<AprilTagHelp> addVisionMeasurement){
        cam = new PhotonCamera(str);
        this.addVisionMeasurement = addVisionMeasurement;
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

        List<PhotonPipelineResult> results = cam.getAllUnreadResults(); 
        if(results.isEmpty()){
            return;
        }
        for(PhotonPipelineResult targetPose : results ){
            
            System.out.println(targetPose);
            System.out.println(targetPose);
            

            Optional<EstimatedRobotPose> optionalEstimPose = (estim.update(targetPose)); 
            
            if(optionalEstimPose.isEmpty()){
                return;
            }
            
            Pose3d estimPose3d ; 
            estimPose3d = optionalEstimPose.get().estimatedPose;

            Pose2d pos = estimPose3d.toPose2d(); // yay :0 im so happy
            double timestamp = targetPose.getTimestampSeconds();
            Matrix<N3, N1> sd;
            
            addVisionMeasurement.accept(new AprilTagHelp(pos, timestamp, sd));
            
        }   
    
       
    }

}
