//Draft
package frc.lib5507.util;

/**
 * Designed to store PID constants in an easily accessible way.
 *
 * @author Finn Frankis
 * @version 10/21/18
 */
public class Gains {
  private double kF;
  private double kP;
  private double kI;
  private double kD;
  private int iZone;

  public Gains(double kF, double kP, double kI, double kD, int iZone) {
    this.kF = kF;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.iZone = iZone;
  }

  public Gains kF(double kF) {
    this.kF = kF;
    return this;
  }

  public Gains kP(double kP) {
    this.kP = kP;
    return this;
  }

  public Gains kI(double kI) {
    this.kI = kI;
    return this;
  }

  public Gains kD(double kD) {
    this.kD = kD;
    return this;
  }

  public Gains iZone(int iZone) {
    this.iZone = iZone;
    return this;
  }

  public double getkF() {
    return kF;
  }

  public double getkP() {
    return kP;
  }

  public double getkI() {
    return kI;
  }

  public double getkD() {
    return kD;
  }

  public int getIZone() {
    return iZone;
  }
}
//Draft