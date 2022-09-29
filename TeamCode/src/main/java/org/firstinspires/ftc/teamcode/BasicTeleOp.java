package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="BasicTeleOp", group = "a")
public class BasicTeleOp extends LinearOpMode {

    private DcMotorEx fL, fR, bL,bR;
    private double flPower, frPower, blPower, brPower;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU imu;
        BNO055IMU.Parameters gyro = new BNO055IMU.Parameters();
        gyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //gyro.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        gyro.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(gyro);

        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

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
            telemetry.addData("turn", rx);
            /*
            flPower = y + x - rx;
            frPower = y - x + rx;
            blPower = y - x - rx;
            brPower = y + x + rx;
            */

            if(rx < .5) {
                flPower = y + x - rx;
                frPower = y - x + rx;
                blPower = y - x - rx;
                brPower = y + x + rx;

            }

            if(rx > .5) {
                flPower = y + x - 1;
                frPower = y - x + 1;
                blPower = y - x - 1;
                brPower = y + x + 1;
                telemetry.update();
                telemetry.addData("Angle", angles.secondAngle);
                telemetry.update();
            }

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

            double flyPower = -1 * gamepad1.right_trigger;

            intake.setPower(flyPower);
        }


    }

}