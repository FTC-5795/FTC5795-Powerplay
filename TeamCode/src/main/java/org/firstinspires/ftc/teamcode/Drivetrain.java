package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drivetrain", group = "a")
public class Drivetrain extends LinearOpMode {

    private DcMotorEx fL, fR, bL,bR;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        fL.setPower(1);
        fR.setPower(1);
        bL.setPower(1);
        bR.setPower(1);
        timer.reset();

        while (timer.time() < 1000) {

        }

        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

}