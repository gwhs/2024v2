package frc.robot;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;

public class RobotTest {

    
    @Test
    void testRobot() {
        Robot main = new Robot();
        Assertions.assertNotNull(main, "robot must not be null");
    }
    @Test
    void testMathFail() {
        final int answer = 4;
        Assertions.assertEquals(answer, 2 + 2, "math test");
    }
}


