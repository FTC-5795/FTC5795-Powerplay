package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//hSlideServo Controls
public class hSlideServoController {

    private Servo hSlideServo;
    private double hSlidePosition;

    public hSlideServoController(HardwareMap hardwareMap) {
        hSlideServo = hardwareMap.get(Servo.class, "hSlideServo");
        hSlideServo.scaleRange(0.1,0.38);
    }

    public void hSlide(boolean rB, boolean lB, double rT, double lT) {
        double target = rT-lT;
        hSlidePosition += 0.01*target;

        if (rB) {
            hSlidePosition = 1;
        }
        else if (lB) {
            hSlidePosition = 0;
        }

        hSlidePosition = 0; //TODO: Remove to unlock horizontal slide from retracted position!

        hSlideServo.setPosition(hSlidePosition);
    }

    public void autoHSlide(double position) {
        hSlideServo.setPosition(position); //1 is extended, 0 is retracted
    }
}
