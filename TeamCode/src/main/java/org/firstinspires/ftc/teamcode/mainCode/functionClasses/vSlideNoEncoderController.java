package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class vSlideNoEncoderController {

    private DcMotorEx vSlideMotor;
    private double vSlidePower;
    boolean hold;

    public vSlideNoEncoderController(HardwareMap hardwareMap) {
        vSlideMotor = hardwareMap.get(DcMotorEx.class, "vSlideMotor");
        vSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void vSlide(boolean dUP, boolean dDOWN) {
        if (dUP) {
            vSlidePower = 0.8;
            vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hold = true;
        }
        else if (dDOWN) {
            vSlidePower = -0.8;
            vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hold = false;
        }
        else {
            if (hold) {
                vSlidePower = 0.05;
            }
            else {
                vSlidePower = 0;
            }
        }
        vSlideMotor.setPower(vSlidePower);
    }
}
