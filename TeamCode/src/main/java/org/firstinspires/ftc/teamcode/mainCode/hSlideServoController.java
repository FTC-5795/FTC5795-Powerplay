package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

//hSlideServo Controls (be sure to start bot with hSlide fully retracted)
public class hSlideServoController extends LinearOpMode {

    private CRServo hSlideServo;
    private double hSlidePower, cumulativeGain = 0;
    private double MAXGain = 100; //Max extension of slide
    private boolean loopActive;

    @Override
    public void runOpMode() throws InterruptedException {
        hSlideServo = hardwareMap.get(CRServo.class, "hSlideServo");
        hSlidePower = gamepad2.right_trigger-gamepad1.left_trigger;

        if (gamepad2.left_bumper) {
            hSlidePower = -1;
        }
        else if (gamepad2.right_bumper) {
            hSlidePower = 1;
        }

        hSlideSafety();
        hSlideServo.setPower(hSlidePower);
    }
    public void autoHSlide(boolean slideExtend) {
        loopActive = true;
        while (loopActive) {
            if (slideExtend) {
                hSlidePower = 1;
            }
            else {
                hSlidePower = -1;
            }
            hSlideSafety();
            hSlideServo.setPower(hSlidePower);
        }
    }
    public void hSlideSafety() {
        cumulativeGain += hSlidePower; //from 0 to MAXGain
        if (cumulativeGain <= 0) {
            cumulativeGain = 0;
            hSlidePower = 0;
            loopActive = false;
        }
        else if (cumulativeGain >= MAXGain) {
            cumulativeGain = MAXGain;
            hSlidePower = 0;
            loopActive = false;
        }
    }
}
