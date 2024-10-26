// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class TriggerTest {
  private GameRobotContainer gameRobotContainer;
  private XboxControllerSim driverControllerSim;
  private XboxControllerSim operatorControllerSim;

  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private PizzaBoxSubsystem pizzaBoxSubsystem;

  private IntakeIOSim intakeIO;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    gameRobotContainer = new GameRobotContainer();
    driverControllerSim = new XboxControllerSim(0);
    operatorControllerSim = new XboxControllerSim(1);

    try {
      Field intakeSubsystemField = GameRobotContainer.class.getDeclaredField("m_IntakeSubsystem");
      intakeSubsystemField.setAccessible(true);
      intakeSubsystem = (IntakeSubsystem) intakeSubsystemField.get(gameRobotContainer);

      Field armSubsystemField = GameRobotContainer.class.getDeclaredField("m_ArmSubsystem");
      armSubsystemField.setAccessible(true);
      armSubsystem = (ArmSubsystem) armSubsystemField.get(gameRobotContainer);

      Field pizzaBoxSubsystemField = GameRobotContainer.class.getDeclaredField("m_PizzaBoxSubsystem");
      pizzaBoxSubsystemField.setAccessible(true);
      pizzaBoxSubsystem = (PizzaBoxSubsystem) pizzaBoxSubsystemField.get(gameRobotContainer);

      Field intakeIOField = IntakeSubsystem.class.getDeclaredField("intakeIO");
      intakeIOField.setAccessible(true);
      intakeIO = (IntakeIOSim) intakeIOField.get(intakeSubsystem);
    } catch (Exception e) {
      e.printStackTrace();
    }

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
  void deployAndRetractIntakeButton() {
    driverControllerSim.setBButton(true);
    driverControllerSim.notifyNewData();
    waitForUpdate(1);
    driverControllerSim.setBButton(false);
    driverControllerSim.notifyNewData();

    waitForUpdate(3);

    assertEquals(IntakeConstants.DOWN_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(true, intakeSubsystem.isDeployed.getAsBoolean());
    assertEquals(true, intakeSubsystem.getSpinSpeed() > 50);

    driverControllerSim.setBButton(true);
    driverControllerSim.notifyNewData();
    waitForUpdate(1);
    driverControllerSim.setBButton(false);
    driverControllerSim.notifyNewData();

    waitForUpdate(3);

    assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(false, intakeSubsystem.isDeployed.getAsBoolean());
    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);
  }

  @Test
  @Order(2)
  void pickNoteFromGroundTest() {
    driverControllerSim.setBButton(true);
    driverControllerSim.notifyNewData();
    waitForUpdate(1);
    driverControllerSim.setBButton(false);
    driverControllerSim.notifyNewData();

    waitForUpdate(3);

    assertEquals(IntakeConstants.DOWN_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(true, intakeSubsystem.isDeployed.getAsBoolean());
    assertEquals(64, intakeSubsystem.getSpinSpeed(), 1);
    assertEquals(0, pizzaBoxSubsystem.getSpeed(), 0.1);

    intakeIO.noteSensorTrue();

    waitForUpdate(2.5);

    assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(ArmConstants.INTAKE_ANGLE, armSubsystem.getArmAngle(), 1);
    assertEquals(-25.4, pizzaBoxSubsystem.getSpeed(), 1);
    assertEquals(79, intakeSubsystem.getSpinSpeed(), 1);

    intakeIO.noteSensorFalse();

    waitForUpdate(5);

    assertEquals(0, pizzaBoxSubsystem.getSpeed(), 0.1);
    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.1);
    
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
