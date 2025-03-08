package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class EagleUtilTest {

  @Test
  public void testClosestTo0_0() {
    EagleUtil.calculateRedReefSetPoints();
    EagleUtil.calculateBlueReefSetPoints();

    Pose2d closest = EagleUtil.closestReefSetPoint(EagleUtil.bluePoses[0], 0);
    Assertions.assertEquals(EagleUtil.bluePoses[0], closest);
  }
}
