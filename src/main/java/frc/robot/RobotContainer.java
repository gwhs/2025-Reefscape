package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.generated.TunerConstants_Comp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import java.util.function.BiConsumer;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);

  private final Telemetry logger =
      new Telemetry(TunerConstants_Comp.kSpeedAt12Volts.in(MetersPerSecond));

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CommandSwerveDrivetrain drivetrain =
      TunerConstants_Comp.createDrivetrain(elevator::getHeightMeters);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  // private final LedSubsystem led = new LedSubsystem();
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final DriveCommand driveCommand =
      new DriveCommand(controller, drivetrain, elevator::getHeightMeters);

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm, groundIntake);

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {
    configureAutonomous();
    configureBindings();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RobotModeTriggers.disabled().onTrue(Commands.parallel(drivetrain.stopDrivetrain()));
    controller.leftBumper().onTrue(collectGroundCoral()).onFalse(resetGroundIntake());
    controller.rightBumper().onTrue(scoreL1()).onFalse(resetGroundIntake());
    // keybind
  }

  public void periodic() {
    robotVisualizer.update();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private void configureAutonomous() {}

  public Command collectGroundCoral() {
    return groundIntake.setAngleAndAmp(
        GroundIntakeConstants.INTAKE_CORAL_ANGLE,
        GroundIntakeConstants.INTAKE_CORAL_AMP,
        GroundIntakeConstants.INTAKE_CORAL_DUTYCYCLE);
  }

  public Command scoreL1() {
    return groundIntake.setAngleAndAmp(
        GroundIntakeConstants.SCORE_CORAL_ANGLE,
        GroundIntakeConstants.SCORE_CORAL_AMP,
        GroundIntakeConstants.SCORE_CORAL_DUTYCYCLE);
  }

  public Command resetGroundIntake() {
    return groundIntake.setAngleAndAmp(0, 0, 0);
  }
}
