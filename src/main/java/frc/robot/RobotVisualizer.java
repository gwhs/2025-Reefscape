// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Add your docs here. */
public class RobotVisualizer {
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH * 5);

//   private static final double kElevatorMinimumLength = 1.5;

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(27.5);

  public Color colorBlue = Color.kFirstBlue;
  public Color colorRed = Color.kFirstRed;

  // Code for the stick figure of each subsystems

  // make the elevator length increase based on height

  // elevator base

  MechanismRoot2d base = panel.getRoot("base", -0.31, 0.0175);
  MechanismLigament2d m_base =
      base.append(new MechanismLigament2d("baseL", 1.3, 0, 10, new Color8Bit(colorBlue)));

  // Code for outer elevator, color coded blue
  MechanismRoot2d root = panel.getRoot("elevator", 0, 0);
  MechanismLigament2d m_elevator =
      root.append(new MechanismLigament2d("elevatorL", 1, 90, 10, new Color8Bit(colorBlue)));

  MechanismRoot2d root2 = panel.getRoot("elevator2", 0.7, 0);
  MechanismLigament2d m_elevator2 =
      root2.append(new MechanismLigament2d("elevatorL2", 1, 90, 10, new Color8Bit(colorBlue)));

  // inner elevator, moving part, color coded red
  MechanismRoot2d root3 = panel.getRoot("elevator3", 0.06, 0.078);
  MechanismLigament2d m_elevator3 =
      root3.append(new MechanismLigament2d("elevatorL3", 1.5, 90, 10, new Color8Bit(colorRed)));

  MechanismRoot2d root4 = panel.getRoot("elevator4", 0.64, 0.078);
  MechanismLigament2d m_elevator4 =
      root4.append(new MechanismLigament2d("elevatorL4", 1.5, 90, 10, new Color8Bit(colorRed)));

  MechanismLigament2d m_elevatorH2 =
      m_elevator3.append(
          new MechanismLigament2d("horizontal2", 0.60, -90, 10, new Color8Bit(colorRed)));
  MechanismLigament2d m_elevatorH3 =
      m_elevator3.append(
          new MechanismLigament2d("horizontal3", 0.31, -90, 10, new Color8Bit(colorRed)));

  MechanismLigament2d m_elevatorH1 =
      root3.append(new MechanismLigament2d("horizontal1", 0.60, 0, 10, new Color8Bit(colorRed)));

  // arm
  MechanismLigament2d m_arm =
      m_elevatorH3.append(
          new MechanismLigament2d(
              "arm", Units.inchesToMeters(20), 90, 10, new Color8Bit(Color.kWhite)));

  public RobotVisualizer(ElevatorSubsystem elevator, ArmSubsystem arm) {
    this.elevator = elevator;
    this.arm = arm;
  }

  public void update() {
    double elevatorHeight = elevator.getHeightMeters();
    double armAngle = arm.getAngle();

    m_arm.setAngle(armAngle + 180);

    m_elevator3.setLength(elevatorHeight + Units.inchesToMeters(37.2));
    m_elevator4.setLength(elevatorHeight + Units.inchesToMeters(37.2));

    SmartDashboard.putData("RobotVisualizer", panel);
  }
}
