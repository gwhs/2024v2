package frc.robot.subsystems.Arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO{
    private SingleJointedArmSim armSim = 
        new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            100,
            SingleJointedArmSim.estimateMOI(.15,5),
            .05,
            Units.degreesToRadians(0),
            Units.degreesToRadians(350),
            true,
            Units.degreesToRadians(90));


    public double getArmEncoderAngle() {
        return Units.radiansToDegrees(armSim.getAngleRads());
    }

    public void setArmSpeed(double speed) {
        speed *=10;
        armSim.setInputVoltage(speed);
    }

    public void update() {
        armSim.update(.020);
    }
}
