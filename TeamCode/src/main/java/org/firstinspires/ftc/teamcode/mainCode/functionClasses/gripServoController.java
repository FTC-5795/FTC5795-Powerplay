package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//cone upright arm controls (be sure to start bot with servo fully upright)
public class gripServoController {

    private Servo gripServo;
    private boolean armDown, spamLock;

    public gripServoController(HardwareMap hardwareMap) {
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        gripServo.setPosition(0.7);
    }

    public void grip(boolean button) {

        if (button && !spamLock) {
            armDown = !armDown;
            toggle();
            spamLock = true;
        }
        else if (!button) {
            spamLock = false;
        }
    }

    public void toggle() {

        if (armDown) {
            gripServo.setPosition(0.82);
        }
        else {
            gripServo.setPosition(0.7);
        }
    }

    public void autoGrip(boolean grab) {
        armDown = grab;
        toggle();
    } //input true for grab, false for release
}
