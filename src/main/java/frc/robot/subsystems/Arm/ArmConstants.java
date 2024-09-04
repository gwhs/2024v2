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
    public static final int ARM_ROTATION_TO_DEGREES = 360;
    public static final double ARM_ENCODER_OFFSET = 18.6;
}
