// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
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
    elevatorIO.update();
    DogLog.log("Elevator/rotation", elevatorIO.getRotation());
    DogLog.log("Elevator/meters", rotationsToMeters(elevatorIO.getRotation()));
    DogLog.log("Elevator/Limit Switch Value (Reverse)", elevatorIO.getReverseLimit());
    DogLog.log("Elevator/Limit Switch Value (Forward)", elevatorIO.getForwardLimit());

    DogLog.log("Elevator/Max Height (meter)", ElevatorConstants.TOP_METER);
  }

  /** Drives the elevator to the give elevation in meters */
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
   */
  public Command homingCommand() {
    return this.runOnce(
            () -> {
              elevatorIO.setVoltage(-3);
            })
        .andThen(Commands.waitUntil(() -> elevatorIO.getReverseLimit()))
        .andThen(
            Commands.runOnce(
                () -> {
                  elevatorIO.setVoltage(0);
                  elevatorIO.setPosition(0);
                }));
  }

  public static double rotationsToMeters(double rotations) {
    return rotations
        / ElevatorConstants.GEAR_RATIO
        * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * 1;
  }

  public static double metersToRotations(double meters) {
    return meters
        / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * ElevatorConstants.GEAR_RATIO
        / 1;
  }

  public double getHeightMeters() {
    return rotationsToMeters(elevatorIO.getRotation());
  }

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
