package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import org.littletonrobotics.junction.Logger;

public class BoreEncoder extends SubsystemBase {
  private final Encoder m_encoder;
  String name;

  // private ShuffleboardTab tab = Shuffleboard.getTab("Encoder");
  // private GenericEntry encoderPosition = tab.add("Encoder Position", 0).getEntry();
  // private GenericEntry encoderRate =
  //     tab.add("Encoder Rate", 0)
  //         .withWidget(BuiltInWidgets.kDial)
  //         .withProperties(Map.of("min", -500, "max", 500))
  //         .getEntry();

  /** Creates a new BoreEncoder. */
  public BoreEncoder(int channel1, int channel2, String name) {
    this.name = name;
    m_encoder = new Encoder(channel1, channel2, false, CounterBase.EncodingType.k4X);

    m_encoder.reset();
    m_encoder.setSamplesToAverage(5);
    m_encoder.setDistancePerPulse(1. / 256.);
    m_encoder.setMinRate(1.0);
  }

  public double getDistance() {
    return m_encoder.getDistance();
  }

  public double getRaw() {

    return m_encoder.getRaw();
  }

  public boolean posDown() {
    double rawAngle = (-m_encoder.getRaw() / 8192. * 360.);
    return Math.abs(rawAngle) < 57;
  }

  public boolean posDown2() {
    double rawAngle = (-m_encoder.getRaw() / 8192. * 360.);
    return Math.abs(rawAngle) < 20;
  }

  public void reset() {
    m_encoder.reset();
  }

  public boolean getStopped() {
    return m_encoder.getStopped();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ticks = m_encoder.get();

    double rawAngle = (-m_encoder.getRaw() / 8192. * 360.);
//    Logger.getInstance().recordOutput(name + "/Angle", rawAngle);

    // SmartDashboard.putNumber("Encoder ticks", ticks);
    // SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    // SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
    // encoderPosition.setDouble(ticks);
    // encoderRate.setDouble(m_encoder.getRate());
  }
}