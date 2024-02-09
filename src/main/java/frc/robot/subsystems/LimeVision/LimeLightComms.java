// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Copied from
 * https://github.com/FRC6854/VIKING/blob/87d32f06d99d272e8859735a67003e81cb4eae8b/src/main/java/viking/vision/limelight/LimelightComm.java
 * All of this code are getters and setters for the
 */

package frc.robot.subsystems.LimeVision;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightComms {
  private String networkTableName;

  public LimeLightComms(String llNetworkTableName) {
    networkTableName = llNetworkTableName;
  }

  /*
  These are possible entries to ***GET***
  tv 	Whether the limelight has any valid targets (0 or 1)
  tx 	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  ty 	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  ta 	Target Area (0% of image to 100% of image)
  ts 	Skew or rotation (-90 degrees to 0 degrees)
  tl 	The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
  tshort 	Sidelength of shortest side of the fitted bounding box (pixels)
  tlong 	Sidelength of longest side of the fitted bounding box (pixels)
  thor 	Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  tvert 	Vertical sidelength of the rough bounding box (0 - 320 pixels)
  getpipe 	True active pipeline index of the camera (0 .. 9)
  tid 	ID of primary AprilTag
  json 	Full JSON dump of targeting results
  tclass 	Class ID of primary neural detector result or neural classifier result

  Array entries
  botpose 	Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
  camtran 	Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
   */

  public double get_entry_double(String entry_name) {
    return NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .getDouble(0);
  }

  public Number get_entry_number(String entry_name) {
    return NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .getNumber(0);
  }

  public double[] get_entry_double_array(String entry_name) {
    return NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .getDoubleArray(new double[0]);
  }

  public Number[] get_entry_number_array(String entry_name) {
    return NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .getNumberArray(new Number[0]);
  }

  /*
  * Possible things to set:
  * ledMode 	Sets limelight’s LED state
   0 	use the LED Mode set in the current pipeline
   1 	force off
   2 	force blink
   3 	force on

   camMode 	Sets limelight’s operation mode
   0 	Vision processor
   1 	Driver Camera (Increases exposure, disables vision processing)

   pipeline 	Sets limelight’s current pipeline
   0 .. 9 	Select pipeline 0..9

   stream 	Sets limelight’s streaming mode
   0 	Standard - Side-by-side streams if a webcam is attached to Limelight
   1 	PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
   2 	PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream

   snapshot 	Allows users to take snapshots during a match
   0 	Reset snapshot mode
   1 	Take exactly one snapshot

   crop 	Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.
   [0] 	X0 - Min or Max X value of crop rectangle (-1 to 1)
   [1] 	X1 - Min or Max X value of crop rectangle (-1 to 1)
   [2] 	Y0 - Min or Max Y value of crop rectangle (-1 to 1)
   [3] 	Y1 - Min or Max Y value of crop rectangle (-1 to 1)
  *
  */

  public void set_entry_double(String entry_name, double value) {
    NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .setDouble(value);
  }

  public void set_entry_number(String entry_name, Number value) {
    NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .setNumber(value);
  }

  public void set_entry_double_array(String entry_name, double[] array) {
    NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .setDoubleArray(array);
  }

  public void set_entry_number_array(String entry_name, Number[] array) {
    NetworkTableInstance.getDefault()
        .getTable(networkTableName)
        .getEntry(entry_name)
        .getNumberArray(new Number[0]);
  }
}
