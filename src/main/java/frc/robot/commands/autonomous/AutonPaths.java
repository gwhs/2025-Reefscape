// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class AutonPaths {
  public static PathPlannerPath SC_F;
  public static PathPlannerPath F_CSP;
  public static PathPlannerPath CSP_E;
  public static PathPlannerPath E_CSP;
  public static PathPlannerPath CSP_D;
  public static PathPlannerPath D_CSP;
  public static PathPlannerPath CSP_C;
  public static PathPlannerPath C_CSP;
  public static PathPlannerPath CSP_B;
  public static PathPlannerPath B_CSP;

  public static PathPlannerPath SL_I;
  public static PathPlannerPath I_CS;
  public static PathPlannerPath CS_J;
  public static PathPlannerPath J_CS;
  public static PathPlannerPath CS_K;
  public static PathPlannerPath K_CS;
  public static PathPlannerPath CS_L;
  public static PathPlannerPath L_CS;
  public static PathPlannerPath CS_A;

  static {
    try {
      SC_F = PathPlannerPath.fromPathFile("SC-F");
      F_CSP = PathPlannerPath.fromPathFile("F-CSP");
      CSP_E = PathPlannerPath.fromPathFile("CSP-E");
      E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      D_CSP = PathPlannerPath.fromPathFile("D-CSP");
      CSP_C = PathPlannerPath.fromPathFile("CSP-C");
      C_CSP = PathPlannerPath.fromPathFile("C-CSP");
      CSP_B = PathPlannerPath.fromPathFile("CSP-B");
      B_CSP = PathPlannerPath.fromPathFile("B-CSP");

      SL_I = PathPlannerPath.fromPathFile("(5CC1) SL-I");
      I_CS = PathPlannerPath.fromPathFile("(5CC1) I-CoralStation");
      CS_J = PathPlannerPath.fromPathFile("(5CC1) CoralStation-J");
      J_CS = PathPlannerPath.fromPathFile("(5CC1) J-CoralStation");
      CS_K = PathPlannerPath.fromPathFile("(5CC1) CoralStation-K");
      K_CS = PathPlannerPath.fromPathFile("(5CC1) K-CoralStation");
      CS_L = PathPlannerPath.fromPathFile("(5CC1) CoralStation-L");
      L_CS = PathPlannerPath.fromPathFile("(5CC1) L-CoralStation");
      CS_A = PathPlannerPath.fromPathFile("(5CC1) CoralStation-A");
    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
