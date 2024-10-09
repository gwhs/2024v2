package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Reaction.ReactionSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbSubsytem;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.swervedrive.Telemetry;
import frc.robot.subsystems.swervedrive.TunerConstants;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

public class GameRobotContainer implements BaseContainer {

  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final PizzaBoxSubsystem m_PizzaBoxSubsystem = new PizzaBoxSubsystem();
  private final ClimbSubsytem m_ClimbSubsystem = new ClimbSubsytem();
  private final ReactionSubsystem m_ReactionSubsystem = new ReactionSubsystem();
  private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

  private final CTRETeleopDrive drive = new CTRETeleopDrive(driverController);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

  public final Trigger teleopEnabled = new Trigger(() -> DriverStation.isTeleopEnabled());

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
    testingLayout.add(scoreSpeaker(160));
    testingLayout.add(scoreSpeaker(230));
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
    teleopEnabled.onTrue(retractIntake());

    /* Driver Controller */
    driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldRelative));
    driverController.a()
        .onTrue(deployIntake())
        .onFalse(retractIntake());

    (driverController.b().and(m_IntakeSubsystem.isDeployed)).debounce(0.1).onTrue(retractIntake());
    (driverController.b().and(m_IntakeSubsystem.isDeployed.negate())).debounce(0.1).onTrue(deployIntake());

    m_IntakeSubsystem.noteTriggered.onTrue(retractIntakePassToPB());

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
    return m_IntakeSubsystem.deployIntake()
        .withName("Deploy Intake");
  }

  public Command retractIntake() {
    return m_IntakeSubsystem.retractIntake()
        .withName("Retract Intake");
  }

  public Command retractIntakePassToPB() {
    return Commands.sequence(
        Commands.parallel(
            m_ArmSubsystem.spinArm(ArmConstants.INTAKE_ANGLE),
            m_IntakeSubsystem.retractIntake()).withTimeout(3),
        Commands.parallel(
            m_IntakeSubsystem.intakeNote(),
            m_PizzaBoxSubsystem.slurp_command(0.5)),
        Commands.waitUntil(m_IntakeSubsystem.noteTriggered.negate()),
        Commands.waitSeconds(1),
        Commands.parallel(
            m_IntakeSubsystem.stopIntake(),
            m_PizzaBoxSubsystem.stopMotor()))
        .withName("Retract Intake and Pass to PB")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command scoreSpeaker(double armAngle) {
    // TODO
    return Commands.none()
        .withName("Score Speaker at " + armAngle);
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
    return Commands.sequence(
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_FLAP).alongWith(m_ClimbSubsystem.motorHalfWay()),
        m_PizzaBoxSubsystem.setFlap(),
        Commands.waitSeconds(0.5),
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_CLIMB),
        m_ClimbSubsystem.motorUp())
        .withName("Prep Climb");
  }

  public Command climbAndScore() {
    return Commands.sequence(
        m_ReactionSubsystem.extendReactionBar(),
        m_ClimbSubsystem.motorDown(),
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_TRAP),
        m_PizzaBoxSubsystem.spit_command(1.0),
        Commands.waitSeconds(2),
        m_PizzaBoxSubsystem.spit_command(0.0))
        .withName("Climb and Score");
  }

  public Command unclimbPartOne() {
  
    return Commands.sequence(
      m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_CLIMB).alongWith(m_ClimbSubsystem.motorHalfWay()),
      m_ClimbSubsystem.motorUp()
    )
        .withName("Unclimb Part One");
  }

  public Command unclimbPartTwo() {
    return Commands.sequence(
      
      m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_CLIMB),
      m_PizzaBoxSubsystem.stopFlap(),
      m_ClimbSubsystem.motorDown())
        .withName("Unclimb Part Two");
  }
}
