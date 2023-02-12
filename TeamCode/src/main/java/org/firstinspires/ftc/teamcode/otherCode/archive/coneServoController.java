package org.firstinspires.ftc.teamcode.otherCode.archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;

@Disabled
//cone upright arm controls (be sure to start bot with servo fully upright)
public class coneServoController {

    private Servo coneServo;
    private boolean armUp, spamLock, spamLock2;
    private double reversePower = 0.5;

    public coneServoController(HardwareMap hardwareMap) {
        coneServo = hardwareMap.get(Servo.class, "coneServo");
        coneServo.setPosition(0.9);
    }

    public void cone(boolean button) {

        if (button && !spamLock) {
            armUp = !armUp;
            toggle(armUp);
            spamLock = true;
        }
        else if (!button) {
            spamLock = false;
        }

    }

    public void toggle(boolean armGoesUp) {

        if (armGoesUp) {
            coneServo.setPosition(0.9);
        }
        else {
            coneServo.setPosition(0.575);
        }
    }
}
