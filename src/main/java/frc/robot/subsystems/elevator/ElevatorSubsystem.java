// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO elevatorIO;

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> elevatorIO.setVoltage(volts.in(Volts)), null, this));

  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOSim();
    } else {
      elevatorIO = new ElevatorIOReal();
    }
  }

  @Override
  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    elevatorIO.update();
    DogLog.log("Elevator/rotation", elevatorIO.getRotation());
    DogLog.log("Elevator/meters", rotationsToMeters(elevatorIO.getRotation()));
    DogLog.log("Elevator/Limit Switch Value (Reverse)", elevatorIO.getReverseLimit());
    DogLog.log("Elevator/Limit Switch Value (Forward)", elevatorIO.getForwardLimit());

    DogLog.log("Elevator/Max Height (meter)", ElevatorConstants.TOP_METER);

    DogLog.log("Loop Time/Elevator", (HALUtil.getFPGATime() - startTime) / 1000);
  }

  /**
   * Drives the elevator to the give elevation in meters
   *
   * @param meters how high?
   * @return move it to that height
   */
  public Command setHeight(double meters) {
    double clampedMeters = MathUtil.clamp(meters, 0, ElevatorConstants.TOP_METER);
    return this.runOnce(
            () -> {
              elevatorIO.setRotation(metersToRotations(clampedMeters));
            })
        .andThen(
            Commands.waitUntil(
                () ->
                    MathUtil.isNear(
                        clampedMeters, rotationsToMeters(elevatorIO.getRotation()), 0.1)));
  }

  /**
   * The idea is that we slowly lower the elevator (by applying negative volt) until the reverse
   * limit switch is triggered then we stop the elevator the motor automatically sets the internal
   * encoder to zero when the reverse limit switch is triggered (see ElevatorIOReal for this config
   * flag)
   *
   * @return run the command
   */
  public Command homingCommand() {
    Command whenNotAtBottom =
        this.runOnce(
                () -> {
                  elevatorIO.setPosition(metersToRotations(ElevatorConstants.TOP_METER));
                  ;
                  elevatorIO.setVoltage(-3);
                })
            .andThen(
                Commands.waitUntil(() -> elevatorIO.getReverseLimit()),
                Commands.runOnce(
                    () -> {
                      elevatorIO.setVoltage(0);
                      elevatorIO.setPosition(0);
                    }));

    Command whenAtBottom =
        Commands.runOnce(
            () -> {
              elevatorIO.setPosition(0);
            });

    return Commands.either(whenNotAtBottom, whenAtBottom, () -> !elevatorIO.getReverseLimit());
  }

  /**
   * @param rotations the amount of rotations
   * @return the rotations in equivalent meters
   */
  public static double rotationsToMeters(double rotations) {
    return rotations
        / ElevatorConstants.GEAR_RATIO
        * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * 1;
  }

  /**
   * @param meters the amount of meters
   * @return the meters in equivalent rotations
   */
  public static double metersToRotations(double meters) {
    return meters
        / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * ElevatorConstants.GEAR_RATIO
        / 1;
  }

  /**
   * @return the height in meters
   */
  public double getHeightMeters() {
    return rotationsToMeters(elevatorIO.getRotation());
  }

  /**
   * @param mode the mode to go to?s
   */
  public void setNeutralMode(NeutralModeValue mode) {
    elevatorIO.setNeutralMode(mode);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
