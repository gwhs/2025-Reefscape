package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class EagleUtilTest {

  @Test
  public void testClosestTo0() {
    EagleUtil.calculateRedReefSetPoints();
    EagleUtil.calculateBlueReefSetPoints();
    EagleUtil.setAllianceOverride(false);

    Pose2d closest = EagleUtil.closestReefSetPoint(EagleUtil.m_bluePoses.get(0), 0);
    Assertions.assertEquals(EagleUtil.m_bluePoses.get(0), closest);

    EagleUtil.setAllianceOverride(true);
    closest = EagleUtil.closestReefSetPoint(EagleUtil.m_redPoses.get(0), 0);
    Assertions.assertEquals(EagleUtil.m_redPoses.get(0), closest);
  }
}
