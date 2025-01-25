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

/** Add your docs here. */
public class RobotVisualizer {
  // private final ArmSubsystem armSubsystem;
  // private final IntakeSubsystem intakeSubsystem;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH * 3);

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);

  public Color color1 = Color.kFirstBlue;
  public Color color2 = Color.kFirstRed;

  // Code for the stick figure of each subsystems

  // elevator base

  MechanismRoot2d base = panel.getRoot("base", -0.31, 0.0175);
  MechanismLigament2d m_base =
      base.append(new MechanismLigament2d("baseL", 1.3, 0, 10, new Color8Bit(color1)));

  // Code for elevator
  MechanismRoot2d root = panel.getRoot("elevator", 0, 0);
  MechanismLigament2d m_elevator =
      root.append(new MechanismLigament2d("elevatorL", 1.5, 90, 10, new Color8Bit(color1)));

  MechanismRoot2d root2 = panel.getRoot("elevator2", 0.7, 0);
  MechanismLigament2d m_elevator2 =
      root2.append(new MechanismLigament2d("elevatorL2", 1.5, 90, 10, new Color8Bit(color1)));

  // inner elevator
  MechanismRoot2d root3 = panel.getRoot("elevator3", 0.04, 0.078);
  MechanismLigament2d m_elevator3 =
      root3.append(new MechanismLigament2d("elevatorL3", 1.5, 90, 10, new Color8Bit(color2)));

  MechanismRoot2d root4 = panel.getRoot("elevator4", 0.66, 0.078);
  MechanismLigament2d m_elevator4 =
      root4.append(new MechanismLigament2d("elevatorL4", 1.5, 90, 10, new Color8Bit(color2)));

  MechanismLigament2d m_elevatorH2 =
      m_elevator3.append(
          new MechanismLigament2d("horizontal2", 0.62, -90, 10, new Color8Bit(color2)));
  MechanismLigament2d m_elevatorH3 =
      m_elevator3.append(
          new MechanismLigament2d("horizontal3", 0.31, -90, 10, new Color8Bit(color2)));

  MechanismLigament2d m_elevatorH1 =
      root3.append(new MechanismLigament2d("horizontal1", 0.62, 0, 10, new Color8Bit(color2)));

  // arm
  MechanismLigament2d arm =
      m_elevatorH3.append(
          new MechanismLigament2d("arm", 0.85, -90, 10, new Color8Bit(Color.kWhite)));

  public void update() {
    SmartDashboard.putData("RobotVisualizer", panel);
  }

  public static void updatePoseEstim() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstim'");
  }
}
