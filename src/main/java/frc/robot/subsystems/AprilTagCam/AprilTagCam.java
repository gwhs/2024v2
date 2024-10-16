// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AprilTagCam;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


/** Add your docs here. */
public class AprilTagCam {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    PhotonCamera cam;

    public AprilTagCam(String str){
        cam = new PhotonCamera(str);
    }

    public void getValue(){
        var list =  cam.getAllUnReadResults();
       
    }

}
