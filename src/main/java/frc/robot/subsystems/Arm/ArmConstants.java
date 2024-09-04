// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ArmConstants {

    public static final int ARM_armMotor_ID = 18;
    public static final String ARM_armMotor_CAN = "rio";
    public static final int ARM_armEncoder_CHANNEL = 0;
    public static final double ARM_kP = 8;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;
    public static final double ARM_VEL = 230 * Math.PI / 180;
    public static final double ARM_ACC = 360 * Math.PI / 180;
}
