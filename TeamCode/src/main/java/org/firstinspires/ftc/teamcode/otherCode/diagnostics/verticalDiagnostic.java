package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled

@TeleOp
public class verticalDiagnostic extends LinearOpMode {

    private DcMotorEx vSlideMotor;
    private double vSlidePower;

    @Override
    public void runOpMode() throws InterruptedException {
        vSlideMotor = hardwareMap.get(DcMotorEx.class, "vSlideMotor");
        vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                vSlidePower = -0.5;
                vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else if (gamepad1.dpad_up) {
                vSlidePower = 0.5;
                vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                vSlidePower = 0.00;
            }

            vSlideMotor.setPower(vSlidePower);
            telemetry.addData("position", -vSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}