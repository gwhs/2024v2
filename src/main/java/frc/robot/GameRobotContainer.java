package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.*;
import frc.robot.Util.NoteSimulator;
import frc.robot.Util.RobotVisualizer;
import frc.robot.commands.swervedrive.CTRETeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Reaction.ReactionSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.PizzaBox.PizzaBoxSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbSubsytem;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;

import java.util.Collections;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class GameRobotContainer implements BaseContainer {

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final PizzaBoxSubsystem m_PizzaBoxSubsystem = new PizzaBoxSubsystem();
  private final ClimbSubsytem m_ClimbSubsystem = new ClimbSubsytem();
  private final ReactionSubsystem m_ReactionSubsystem = new ReactionSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CTRETeleopDrive drive = new CTRETeleopDrive(driverController, drivetrain);
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public final Trigger teleopEnabled = new Trigger(() -> DriverStation.isTeleopEnabled());

  private final RobotVisualizer robotVisualizer;

  public GameRobotContainer() {
    DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureNt(true).withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    drivetrain.setDefaultCommand(drive);
    configureBindings();
    configureAutonomous();

    drivetrain.registerTelemetry(logger::telemeterize);

    robotVisualizer = new RobotVisualizer(m_ArmSubsystem, m_IntakeSubsystem);

    SmartDashboard.putData("Robot Commands/Deploy Intake", deployIntake());
    SmartDashboard.putData("Robot Commands/Retract Intake", retractIntake());
    SmartDashboard.putData("Robot Commands/Retract Intake and Pass to PB", retractIntakePassToPB());
    SmartDashboard.putData("Robot Commands/Score Speaker 160", scoreSpeaker(160));
    SmartDashboard.putData("Robot Commands/Score Speaker 230", scoreSpeaker(230));
    SmartDashboard.putData("Robot Commands/Score Amp", scoreAmp());
    SmartDashboard.putData("Robot Commands/Source Intake", sourceIntake());
    SmartDashboard.putData("Robot Commands/Prep Climb", prepClimb());
    SmartDashboard.putData("Robot Commands/Climb and Score", climbAndScore());
    SmartDashboard.putData("Robot Commands/Unclimb Part One", unclimbPartOne());
    SmartDashboard.putData("Robot Commands/Unclimb Part Two", unclimbPartTwo());

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  private void configureBindings() {
    // Reset Robot
    teleopEnabled.onTrue(retractIntake());

    /* Driver Controller */
    driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
    // driverController.a()
    //     .onTrue(deployIntake())
    //     .onFalse(retractIntake());

    // (driverController.b().and(m_IntakeSubsystem.isDeployed)).debounce(0.1).onTrue(retractIntake());
    // (driverController.b().and(m_IntakeSubsystem.isDeployed.negate())).debounce(0.1).onTrue(deployIntake());

    //m_IntakeSubsystem.noteTriggered.onTrue(retractIntakePassToPB());

    driverController.a().whileTrue(Commands.startEnd(
      () -> drive.faceAmp = true,
      () -> drive.faceAmp = false).withName("Face Amp"));

    driverController.a().and(driverController.rightTrigger()).onTrue(scoreAmp()).onFalse(resetArm());
    driverController.y().whileTrue(Commands.startEnd(
        () -> drive.faceSpeaker = true,
        () -> drive.faceSpeaker = false).withName("Face Speaker"));

    driverController.y().and(driverController.rightTrigger()).onTrue(scoreSpeaker(160)).onFalse(resetArm());

  
  //source intake
    driverController.x().whileTrue(Commands.startEnd(
      () -> drive.isHeadingLock = true,
      () -> drive.isHeadingLock = false).withName("Source Intake"));

    driverController.x().and(driverController.rightTrigger()).onTrue(sourceIntake()).onFalse(resetArm());
   }
   

  private void configureAutonomous() {
    autoChooser.setDefaultOption("S3-Leave", new S3Leave(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));

    autoChooser.addOption("S1-Leave", new S1Leave(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));
    autoChooser.addOption("S2-Leave", new S1Leave(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));
    autoChooser.addOption("S3-C5", new S3_C5(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));
    autoChooser.addOption("S3-C5-C4", new S3_C5_C4(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));
    autoChooser.addOption("S3-C5-C4_optimized",
        new S3_C5_C4_optimized(this, m_ArmSubsystem, m_IntakeSubsystem, m_PizzaBoxSubsystem));

    // TODO: add more autonomous routines

    SmartDashboard.putData("autonomous", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   **/
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Override
  public void periodic() {
    robotVisualizer.update();
    NoteSimulator.update(drivetrain.getState().Pose, m_ArmSubsystem.getArmAngle());
  }

  public Command deployIntake() {
    return m_IntakeSubsystem.deployIntake()
        .withName("Deploy Intake");
  }
  public Command resetArm() {
    return Commands.parallel(
      m_ArmSubsystem.spinArm(90),
      m_PizzaBoxSubsystem.spit_command(0).andThen(m_PizzaBoxSubsystem.stopKicker())
    .withName("Reset Arm"));
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
        Commands.waitSeconds(1).alongWith(NoteSimulator.intakeNote()),
        Commands.parallel(
            m_IntakeSubsystem.stopIntake(),
            m_PizzaBoxSubsystem.stopMotor()))
        .withName("Retract Intake and Pass to PB")
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command scoreSpeaker(double armAngle) {
    return Commands.sequence(
        m_ArmSubsystem.spinArm(armAngle)
            .deadlineFor(m_PizzaBoxSubsystem.speedyArm_Command(() -> m_ArmSubsystem.getArmAngle())).withTimeout(2),
        m_PizzaBoxSubsystem.spit_command(1),
        Commands.waitUntil(() -> m_PizzaBoxSubsystem.getVelocity() >= 80).withTimeout(2),
        m_PizzaBoxSubsystem.setKicker()
            .alongWith(Commands.defer(
                () -> NoteSimulator.launchNote(10, drivetrain.getState().Speeds, drivetrain.getState().Pose, armAngle),
                Collections.emptySet())),
        Commands.waitSeconds(0.5),
        m_PizzaBoxSubsystem.stopKicker(),
        m_ArmSubsystem.spinArm(90).alongWith(m_PizzaBoxSubsystem.stopMotor()).withTimeout(0))
        .withName("Score Speaker at " + armAngle);
  }

  public Command scoreAmp() {
    return Commands.sequence(
        m_ArmSubsystem.spinArm(ArmConstants.AMP_ANGLE),
        m_PizzaBoxSubsystem.spit_command(1),
        m_PizzaBoxSubsystem.setKicker(),
        Commands.waitSeconds(0.5),
        m_PizzaBoxSubsystem.stopKicker(),
        m_PizzaBoxSubsystem.stopMotor(),
        m_ArmSubsystem.spinArm(90))
        .withName("scoreAmp");
  }

  public Command sourceIntake() {
    return Commands.sequence(
        m_ArmSubsystem.spinArm(160)).alongWith(m_PizzaBoxSubsystem.slurp_command(0.5))
        .withName("Source Intake");
  }

  public Command prepClimb() {
    return Commands.sequence(
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_FLAP).alongWith(
            Commands.waitUntil(() -> m_ArmSubsystem.getArmAngle() >= 200).andThen(m_ClimbSubsystem.motorHalfWay())),
        m_PizzaBoxSubsystem.setFlap(),
        Commands.waitSeconds(0.5),
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_CLIMB),
        m_ClimbSubsystem.motorUp())
        .withName("Prep Climb");
  }

  public Command climbAndScore() {
    return Commands.sequence(
        m_ReactionSubsystem.extendReactionBar(),
        m_ClimbSubsystem.motorDown().withTimeout(3),
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_TRAP),
        m_PizzaBoxSubsystem.spit_command(0.8),
        Commands.waitSeconds(2),
        m_PizzaBoxSubsystem.spit_command(0.0))
        .withName("Climb and Score");
  }

  public Command unclimbPartOne() {

    return Commands.sequence(
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_CLIMB),
        m_ClimbSubsystem.motorUp())
        .withName("Unclimb Part One");
  }

  public Command unclimbPartTwo() {
    return Commands.sequence(
        m_ClimbSubsystem.motorDown().withTimeout(3),
        m_ArmSubsystem.spinArm(ArmConstants.ARM_ANGLE_FLAP),
        m_PizzaBoxSubsystem.stopFlap(),
        Commands.waitSeconds(1),
        m_ArmSubsystem.spinArm(90),
        m_ReactionSubsystem.retractReactionBar())
        .withName("Unclimb Part Two");
  }
}
