package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;

//cone upright arm controls (be sure to start bot with servo fully upright)
public class coneServoController {

    private Servo coneServo;
    private boolean armUp, spamLock, spamLock2;
    //SampleMecanumDrive drive = new SampleMecanumDrive(HardwareMap);
    private double reversePower = 0.5;

    public coneServoController(HardwareMap hardwareMap) {
        coneServo = hardwareMap.get(Servo.class, "coneServo");
        coneServo.setPosition(0.9);
    }

    public void cone(boolean button, boolean autoButton) {

        if (button && !spamLock) {
            armUp = !armUp;
            toggle();
            spamLock = true;
        }
        else if (!button) {
            spamLock = false;
        }

        if (autoButton && !spamLock2) {
            if (armUp) {
                autoToggle();
            }
        }
        else if (!autoButton) {
                spamLock2 = false;
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

    public void autoToggle() {
        armUp = false;
        toggle();

        for (int i = 0; i < 10; i++) {
            //drive.setMotorPowers(-reversePower, -reversePower, -reversePower, -reversePower);
        } //adjust i < 10 for reverse time
        //drive.setMotorPowers(0,0,0,0);

        armUp = true;
        toggle();
    }
}
