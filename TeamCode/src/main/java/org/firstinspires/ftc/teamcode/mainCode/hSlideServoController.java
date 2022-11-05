package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//hSlideServo Controls
public class hSlideServoController extends LinearOpMode {

    private Servo hSlideServo;
    private double hSlidePosition;

    @Override
    public void runOpMode() throws InterruptedException {
        hSlideServo = hardwareMap.get(Servo.class, "hSlideServo");
        hSlideServo.scaleRange(0.1,0.38);
        double target = gamepad2.right_trigger-gamepad2.left_trigger;
        hSlidePosition += 0.05*target;

        if (gamepad2.right_bumper) {
            hSlidePosition = 1;
        }
        else if (gamepad2.left_bumper) {
            hSlidePosition = 0;
        }
        hSlideServo.setPosition(hSlidePosition);
    }
    public void autoHSlide(double position) {
        hSlideServo = hardwareMap.get(Servo.class, "hSlideServo");
        hSlideServo.scaleRange(0.1,0.38);
        hSlideServo.setPosition(position);
    }
}
