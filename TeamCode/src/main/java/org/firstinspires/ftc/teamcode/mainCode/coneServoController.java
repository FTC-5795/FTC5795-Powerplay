package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

//cone upright arm controls (be sure to start bot with servo fully upright)
public class coneServoController extends LinearOpMode {

    private CRServo coneServo;
    private double conePower, cumulativeGain = 0;
    private boolean spamLock, extendArm;
    private double MAXGain = 100; //Max position of arm

    @Override
    public void runOpMode() throws InterruptedException {
        coneServo = hardwareMap.get(CRServo.class, "coneServo");

        if (gamepad2.b && !spamLock) {
            spamLock = true;
            extendArm = !extendArm;
        }
        else if (!gamepad2.b) {
            spamLock = false;
        }

        toggleArm();
    }
    public void toggleArm() {
        if (extendArm) {
            conePower = 1;
        }
        else {
            conePower = -1;
        }
        coneSafety();
        coneServo.setPower(conePower);
    }
    public void coneSafety() {
        cumulativeGain += conePower;
        if (cumulativeGain <= 0) {
            cumulativeGain = 0;
            conePower = 0;
        }
        else if (cumulativeGain >= MAXGain) {
            cumulativeGain = MAXGain;
            conePower = 0;
        }
    }
}
