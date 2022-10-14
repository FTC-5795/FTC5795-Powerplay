package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Currently implemented for two dead wheel odometry

Notes: -Start position is init position as (0,0) and 90 degrees (Unit Circle) as forward
-16 bit overflow error when using getVelocity function on Rev Encoders means velocity as to be calculated through getCurrentPosition function
 */

@TeleOp

public class TeleOpOdometry extends LinearOpMode {

    //Odometry pod electronic variables
    public static double ticksPerRevolution = 8192;
    public static double wheelRadius = 0.5; //inches
    public static double gearRatio = 1;

    //Odometry pod placement variables
    public static double parallelX = 0; //inches
    public static double parallelY = 0; //inches
    public static double perpendicularX = 0; //inches
    public static double perpendicularY = 0; //inches

    //Motor & Odometry declarations
    private DcMotorEx fL, fR, bL, bR;
    private double fLPower, fRPower, bLPower, bRPower;
    private DcMotorEx perpendicularEncoder, parallelEncoder;
    private double perpendicularDistance, parallelDistance;
    private double oldPerpendicularDistance, oldParallelDistance;
    private Pose2d position = new Pose2d(0, 0, 90); //Start position

    //Other declarations
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry.addData("System:", "calibrating...");
        telemetry.update();

        //imu Mapping
        BNO055IMU imu;
        BNO055IMU.Parameters gyro = new BNO055IMU.Parameters();
        gyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //gyro.calibrationDataFile = "AdafruitIMUCalibration.json"; //see the calibration sample op mode
        gyro.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(gyro);

        //Motor Mapping
        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        bL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        bR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Link Odometry Pods to Motor Ports (Motors and Pods not related)
        perpendicularEncoder = fL;
        parallelEncoder = bL;

        telemetry.addData("System:", "calibration complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Tele-Op Code
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            fLPower = y + x - rx;
            fRPower = y - x + rx;
            bLPower = y - x - rx;
            bRPower = y + x + rx;

            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            //Odometry Code
            oldParallelDistance = parallelDistance;
            oldPerpendicularDistance = perpendicularDistance;

            parallelDistance = parallelEncoder.getCurrentPosition();
            perpendicularDistance = perpendicularEncoder.getCurrentPosition();

            double parallelChange = parallelDistance - oldParallelDistance;
            double perpendicularChange = perpendicularDistance - oldPerpendicularDistance;
            if (imu.getAngularOrientation().firstAngle>=0) {
//                position.component3() = imu.getAngularOrientation().firstAngle + 90; //facing forward
            } else if (imu.getAngularOrientation().firstAngle<0) {
//                position.component3() = imu.getAngularOrientation().firstAngle + 450; //facing forward and positive
            }

        }
    }

}
