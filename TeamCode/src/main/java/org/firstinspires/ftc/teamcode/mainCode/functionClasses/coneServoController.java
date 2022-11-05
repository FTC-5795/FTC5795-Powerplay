package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//cone upright arm controls (be sure to start bot with servo fully upright)
public class coneServoController {

    private Servo coneServo;
    private boolean armUp, spamLock;

    public coneServoController(HardwareMap hardwareMap) {
        coneServo = hardwareMap.get(Servo.class, "coneServo");
        coneServo.setPosition(0.9);
    }

    public void cone(boolean button) {

        if (button && !spamLock) {
            armUp = !armUp;
            toggle();
            spamLock = true;
        }
        else if (!button) {
            spamLock = false;
        }
    }

    public void toggle() {

        if (armUp) {
            coneServo.setPosition(0.9);
        }
        else {
            coneServo.setPosition(0.575);
        }
    }
}
