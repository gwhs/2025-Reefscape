package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class EagleUtilTest {

  @BeforeAll
  public static void setupEagleUtils() {
    EagleUtil.calculateRedReefSetPoints();
    EagleUtil.calculateBlueReefSetPoints();
  }

  @Test
  public void testClosestTo0() {

    EagleUtil.setAllianceOverride(false);
    Pose2d closest = EagleUtil.closestReefSetPoint(EagleUtil.m_bluePoses.get(0), 0);
    Assertions.assertEquals(EagleUtil.m_bluePoses.get(0), closest);

    EagleUtil.setAllianceOverride(true);
    closest = EagleUtil.closestReefSetPoint(EagleUtil.m_redPoses.get(0), 0);
    Assertions.assertEquals(EagleUtil.m_redPoses.get(0), closest);
  }

  @Test
  public void testIndexTest() {

    EagleUtil.setAllianceOverride(false);
    for (int i = 0; i < EagleUtil.m_bluePoses.size(); i++) {
      int index = EagleUtil.findClosestReefIndex(EagleUtil.m_bluePoses.get(i));
      Assertions.assertEquals(i, index);
    }

    EagleUtil.setAllianceOverride(true);
    for (int i = 0; i < EagleUtil.m_redPoses.size(); i++) {
      int index = EagleUtil.findClosestReefIndex(EagleUtil.m_redPoses.get(i));
      Assertions.assertEquals(i, index);
    }
  }

  // to show that tests can fail on build, uncomment this below
  //@Test
  public void testFail() {
    Assertions.assertEquals(1, 2);
  }
}
