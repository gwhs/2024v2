package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsytem;
import frc.robot.subsystems.Reaction.ReactionSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.Telemetry;
import frc.robot.subsystems.swervedrive.TunerConstants;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

public class GameRobotContainer implements BaseContainer {

  CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final PizzaBoxSubsystem m_PizzaBoxSubsystem = new PizzaBoxSubsystem();
  private final ClimbSubsytem m_ClimbSubsystem = new ClimbSubsytem();
  private final ReactionSubsystem m_ReactionSubsystem = new ReactionSubsystem();
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance(); // My drivetrain

  private final CTRETeleopDrive drive = new CTRETeleopDrive(driverController);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

  public GameRobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("Hajel middle bottom 2");

    drivetrain.setDefaultCommand(drive);
    configureBindings();

    drivetrain.registerTelemetry(logger::telemeterize);

    /*
     * Put composite commands to shuffleboard
     */
    ShuffleboardTab testingTab = Shuffleboard.getTab("Testing");
    ShuffleboardLayout testingLayout = testingTab.getLayout("Commands", BuiltInLayouts.kList)
        .withSize(2, 5)
        .withProperties(Map.of("Label Position", "HIDDEN"));

    testingLayout.add(deployIntake());
    testingLayout.add(retractIntake());
    testingLayout.add(retractIntakePassToPB());
    testingLayout.add(scoreSpeaker(() -> 160));
    testingLayout.add(scoreSpeaker(() -> 230));
    testingLayout.add(scoreAmp());
    testingLayout.add(sourceIntake());
    testingLayout.add(prepClimb());
    testingLayout.add(climbAndScore());
    testingLayout.add(unclimbPartOne());
    testingLayout.add(unclimbPartTwo());

    /*
     * Put Command Scheduler and subsystems to shuffleboard
     */
    testingTab.add(CommandScheduler.getInstance()).withSize(3, 2);
    testingTab.add(m_PizzaBoxSubsystem);
    testingTab.add(m_ReactionSubsystem);
    testingTab.add(m_IntakeSubsystem);
    testingTab.add(m_ArmSubsystem);
    testingTab.add(m_ClimbSubsystem);
  }

  private void configureBindings() {
    /* Reset Robot */



    /* Driver Controller */



    /* Operator Controllers */



    /* Other Triggers */

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command deployIntake() {
    // TODO
    return Commands.none()
        .withName("Deploy Intake");
  }

  public Command retractIntake() {
    // TODO
    return Commands.none()
        .withName("Retract Intake");
  }

  public Command retractIntakePassToPB() {
    // TODO
    return Commands.none()
        .withName("Retract Intake and Pass to PB");
  }

  public Command scoreSpeaker(DoubleSupplier armAngle) {
    // TODO
    return Commands.none()
        .withName("Score Speaker at " + armAngle.getAsDouble());
  }

  public Command scoreAmp() {
    // TODO
    return Commands.none()
        .withName("Score Amp");
  }

  public Command sourceIntake() {
    // TODO
    return Commands.none()
        .withName("Source Intake");
  }

  public Command prepClimb() {
    // TODO
    return Commands.none()
        .withName("Prep Climb");
  }

  public Command climbAndScore() {
    // TODO
    return Commands.none()
        .withName("Climb and Score");
  }

  public Command unclimbPartOne() {
    // TODO
    return Commands.none()
        .withName("Unclimb Part One");
  }

  public Command unclimbPartTwo() {
    // TODO
    return Commands.none()
        .withName("Unclimb Part Two");
  }
}
