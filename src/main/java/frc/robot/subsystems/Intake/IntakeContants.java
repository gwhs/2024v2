// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeContants {
    public static final int INTAKE_ARM_ID = 55;
    public static final String INTAKE_ARM_CAN = "CAN_Network";

    public static final int INTAKE_SPIN_ID = 20;
    public static final String INTAKE_SPIN_CAN = "rio";

    public static final int INTAKE_ENCODER_CHANNEL_ID = 9;
    public static final int INTAKE_NOTE_SENSOR_CHANNEL_ID = 8;

    public static final double kP = 0.03;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kVel = 300;
    public static final double kAcc = 300;

    public static final double ENCODER_OFFSET = 170;

    public static final double UP_POSITION = 92;
    public static final double DOWN_POSITION = 0;
}
