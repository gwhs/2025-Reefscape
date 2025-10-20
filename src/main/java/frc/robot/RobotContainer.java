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
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
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

    controller.a().onTrue(loadArmCoral());
    controller.b().onTrue(scoreL4());
    controller.x().onTrue(scoreL2());
    controller.y().onTrue(scoreL3());
  }

  public Command scoreL4() {
    return Commands.sequence(
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.L4_PREP_POSITION),
            arm.setAngle(ArmConstants.L4_PREP_POSITION).withTimeout(2)),
        endEffector.setVoltage(EndEffectorConstants.VOLTAGE_L4),
        Commands.waitSeconds(0.05),
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.STOW_METER),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE),
            endEffector.setVoltage(0)));
  }

  public Command scoreL3() {
    return Commands.sequence(
        Commands.parallel(
                elevator.setHeight(ElevatorConstants.L3_PREP_POSITION),
                arm.setAngle(ArmConstants.L3_PREP_POSITION))
            .withTimeout(2),
        endEffector.setVoltage(EndEffectorConstants.VOLTAGE_L3),
        Commands.waitSeconds(0.05),
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.STOW_METER),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE),
            endEffector.setVoltage(0)));
  }

  public Command scoreL2() {
    return Commands.sequence(
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.L2_PREP_POSITION),
            arm.setAngle(ArmConstants.L2_PREP_POSITION).withTimeout(2)),
        endEffector.setVoltage(EndEffectorConstants.VOLTAGE_L2),
        Commands.waitSeconds(0.05),
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.STOW_METER),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE),
            endEffector.setVoltage(0)));
  }

  public Command loadArmCoral() {
    return Commands.sequence(
        Commands.parallel(
            endEffector.intake(),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE),
            elevator.setHeight(ElevatorConstants.INTAKE_METER)),
        Commands.waitSeconds(1),
        endEffector.holdCoral());
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
}
