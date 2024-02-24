// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class UtilMotor {

    public static void configMotor(TalonFX motor) {
        configMotor(motor, 0.11, 0.5, 0.0001, 0.12, 8, 40);
    }

    public static void configMotor(TalonFX motor, double kP, double kI, double kD, double kV, int peakVoltage, int peakCurrent) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = kP; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = kI; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = kD; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = kV; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = peakVoltage;
    configs.Voltage.PeakReverseVoltage = -peakVoltage;

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = peakCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -peakCurrent;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = motor.getConfigurator().apply(configs);
      if (motorStatus.isOK()) break;
    }
    if(!motorStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + motorStatus.toString());
    }
    }
}