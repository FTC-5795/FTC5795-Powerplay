package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BasicTeleOp", group = "a")
public class BasicTeleOp extends LinearOpMode {

    private DcMotorEx fL, fR, bL,bR;
    private double flPower, frPower, blPower, brPower;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            flPower = y + x - rx;
            frPower = y - x + rx;
            blPower = y - x - rx;
            brPower = y + x + rx;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            flPower /= denominator;
            frPower /= denominator;
            blPower /= denominator;
            brPower /= denominator;

            fL.setPower(flPower);
            fR.setPower(frPower);
            bL.setPower(blPower);
            bR.setPower(brPower);

            DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

            double fly = gamepad1.right_trigger;

            intake.setPower(fly);
        }


    }

}