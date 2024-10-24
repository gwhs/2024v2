// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class IntakeSubsystemTest {
  private static final IntakeIOSim intakeIO = new IntakeIOSim();
  private static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intakeIO);

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
  void deployCommand() {
    intakeSubsystem.deployIntake().schedule();

    waitForUpdate(2);

    assertEquals(IntakeConstants.DOWN_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(true, intakeSubsystem.isDeployed.getAsBoolean());
    assertEquals(true, intakeSubsystem.getSpinSpeed() > 50);
  }

  @Test
  void retractCommand() {
    intakeSubsystem.retractIntake().schedule();

    waitForUpdate(3);

    assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(false, intakeSubsystem.isDeployed.getAsBoolean());
    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);
  }

  @Test
  void intakeCommand() {
    intakeSubsystem.intakeNote().schedule();

    waitForUpdate(2);

    assertEquals(true, intakeSubsystem.getSpinSpeed() > 70);
  }

  @Test
  void stopIntakeCommand() {
    intakeSubsystem.stopIntake().schedule();

    waitForUpdate(3);

    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);
  }

  @Test
  void encoderDisconnected() {
    deployCommand();
    retractCommand();
    intakeCommand();
    stopIntakeCommand();

    intakeIO.disconnectEncoder();

    intakeSubsystem.deployIntake().schedule();

    waitForUpdate(2);

    assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);

    intakeSubsystem.retractIntake().schedule();

    waitForUpdate(2);

    assertEquals(IntakeConstants.UP_POSITION, intakeSubsystem.getIntakeArmAngle(), 0.1);
    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);

    intakeSubsystem.intakeNote().schedule();

    waitForUpdate(2);

    assertEquals(0, intakeSubsystem.getSpinSpeed(), 0.01);

    intakeIO.connectEncoder();
    deployCommand();
    retractCommand();
    intakeCommand();
    stopIntakeCommand();
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
