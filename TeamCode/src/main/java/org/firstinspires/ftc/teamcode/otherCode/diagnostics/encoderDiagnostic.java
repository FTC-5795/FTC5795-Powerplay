package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp
public class encoderDiagnostic extends LinearOpMode {

    private DcMotorEx left, right, back;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotorEx.class, "leftFront");
        right = hardwareMap.get(DcMotorEx.class, "rightFront");
        back = hardwareMap.get(DcMotorEx.class, "leftRear");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("active", "active");
            telemetry.addData("left", left.getCurrentPosition());
            telemetry.addData("right", right.getCurrentPosition());
            telemetry.addData("back", back.getCurrentPosition());
            telemetry.update();
        }


    }
}
