package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

//grip servo controls (be sure to start cone gripper in clamped position)
public class gripServoController extends LinearOpMode {

    private CRServo gripServo;
    private boolean spamLock = false, clampGrip = true, loopActive = false;
    private double MAXGain; //Maximum retraction of clamp
    private double gripPower = 0, cumulativeGain = MAXGain;


    @Override
    public void runOpMode() throws InterruptedException {
        gripServo = hardwareMap.get(CRServo.class, "gripServo");

        if (gamepad2.a && !spamLock) {
            spamLock = true;
            clampGrip = !clampGrip;
        }
        else if (!gamepad2.a) {
            spamLock = false;
        }

        toggleClamp();
    }
    public void autoGrip(boolean clampCone) {
        clampGrip = clampCone;
        loopActive = true;
        while (loopActive) {
            toggleClamp();
        }
    }
    public void toggleClamp() {
        if (clampGrip) {
            gripPower = 1;
        }
        else {
            gripPower = -1;
        }
        gripSafety();
        gripServo.setPower(gripPower);
    }
    public void gripSafety() {
        cumulativeGain += gripPower;
        if (cumulativeGain <= 0) {
            cumulativeGain = 0;
            gripPower = 0;
            loopActive = false;
        }
        else if (cumulativeGain >= MAXGain) {
            cumulativeGain = MAXGain;
            gripPower = 0;
            loopActive = false;
        }
    }
}
