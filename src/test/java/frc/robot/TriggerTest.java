// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TriggerTest {
  private static GameRobotContainer gameRobotContainer;
  private static XboxControllerSim driverControllerSim = new XboxControllerSim(0);
  private static XboxControllerSim operatorControllerSim = new XboxControllerSim(1);

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    gameRobotContainer = new GameRobotContainer();

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void disable() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  @Order(1)
  void deployIntakeButton() {
    try {
      Field intakeSubsystemField = GameRobotContainer.class.getDeclaredField("m_IntakeSubsystem");
      intakeSubsystemField.setAccessible(true);
      IntakeSubsystem intakeSubsystem = (IntakeSubsystem) intakeSubsystemField.get(gameRobotContainer);

      driverControllerSim.setBButton(true);
      driverControllerSim.notifyNewData();
      waitForUpdate(0.5);
      driverControllerSim.setBButton(false);
      driverControllerSim.notifyNewData();

      waitForUpdate(2);

      assertEquals(IntakeConstants.DOWN_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
      assertEquals(true, intakeSubsystem.isDeployed.getAsBoolean());
      assertEquals(true, intakeSubsystem.getSpinSpeed() > 50);

      driverControllerSim.setBButton(true);
      driverControllerSim.notifyNewData();
      waitForUpdate(0.5);
      driverControllerSim.setBButton(false);
      driverControllerSim.notifyNewData();

      waitForUpdate(2);

      assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
      assertEquals(false, intakeSubsystem.isDeployed.getAsBoolean());
      assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);

    } catch (Exception e) {
      e.printStackTrace();
    }

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
