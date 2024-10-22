// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSubsystem.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem.ClimbSubsytem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimbSubsystemTest {
  static ClimbSubsytem climbSubsystem = new ClimbSubsytem();

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void motorUpCommand() {
    climbSubsystem.motorUp().schedule();

    waitForUpdate(4);

    assertEquals(ClimbConstants.LEFT_UP_POSITION, climbSubsystem.getLeftMotorPosition(), 0.5);
    assertEquals(ClimbConstants.RIGHT_UP_POSITION, climbSubsystem.getRightMotorPosition(), 0.5);
  }

  @Test
  void motorDownCommand() {
    climbSubsystem.motorDown().schedule();

    waitForUpdate(4);

    assertEquals(ClimbConstants.LEFT_DOWN_POSITION, climbSubsystem.getLeftMotorPosition(), 0.5);
    assertEquals(ClimbConstants.RIGHT_DOWN_POSITION, climbSubsystem.getRightMotorPosition(), 0.5);
  }

  @Test
  void motorHalfWayCommand() {
    climbSubsystem.motorHalfWay().schedule();

    waitForUpdate(4);

    assertEquals(ClimbConstants.LEFT_UP_POSITION/2, climbSubsystem.getLeftMotorPosition(), 0.5);
    assertEquals(ClimbConstants.RIGHT_UP_POSITION/2, climbSubsystem.getRightMotorPosition(), 0.5);
  }

  private static void waitForUpdate(double seconds) {
    try {
      int updateRateMs = 1;

      double numOfLoops = Units.secondsToMilliseconds(seconds) / 20;

      for (int i = 0; i < numOfLoops; ++i) {
        CommandScheduler.getInstance().run();
        Thread.sleep(updateRateMs);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
