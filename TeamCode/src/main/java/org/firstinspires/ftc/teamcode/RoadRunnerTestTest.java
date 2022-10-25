package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.junit.Test;

public class RoadRunnerTestTest {

    @Test
    public void testPIDControl() {
        RoadRunnerTest PIDtest = new RoadRunnerTest();

        double target = 100;
        double state = 0;
        double output = PIDtest.PIDControl(target, state);
        assertEquals(0,output,0);
    }

}