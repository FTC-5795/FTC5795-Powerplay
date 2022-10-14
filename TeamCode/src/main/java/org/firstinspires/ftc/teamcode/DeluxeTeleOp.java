package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class DeluxeTeleOp extends LinearOpMode {

    private DcMotorEx fL, fR, bL, bR;
    private double fLPower, fRPower, bLPower, bRPower;
    private double lockAngle, orientation, friction; //for drift physics
    private boolean positionalRotationMode; //for ninety degree turn code (boolean)
    private double targetAngle; //for ninety degree turn code (double)
    private double integralSum, derivative, error, previousError; //for PID control (dynamic)
    private static double Kp = 0.5; //
    private static double Ki = 0; //
    private static double Kd = 0.5; //
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{

        BNO055IMU imu;
        BNO055IMU.Parameters gyro = new BNO055IMU.Parameters();
        gyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //gyro.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        gyro.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(gyro);

        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            //Dynamic variables with precision code (sqrt)
            double y = Math.sqrt(Math.abs(-gamepad1.left_stick_y));
            double x = 1.1 * Math.sqrt(Math.abs(gamepad1.left_stick_x));
            double rx = gamepad1.right_stick_x;
            if (-gamepad1.left_stick_y<0) {
                y = -y;
            }
            if (gamepad1.left_stick_x<0) {
                x = -x;
            }

            //Orientation code 0-360 degrees
            if (imu.getAngularOrientation().firstAngle>=0) {
                orientation = imu.getAngularOrientation().firstAngle;
            }
            else if (imu.getAngularOrientation().firstAngle<0) {
                orientation = imu.getAngularOrientation().firstAngle + 360;
            }

            //Drivetrain Code
            if (gamepad1.left_bumper) {
                //Drift Code
                y = -gamepad1.left_stick_y * Math.sin(Math.toRadians(lockAngle - orientation + 90));
                x = 1.1 * -gamepad1.left_stick_y * Math.cos(Math.toRadians(lockAngle - orientation + 90));
                friction = Math.max(friction - 0, 0); //friction coefficient
                fLPower = friction * (y + x) - rx;
                fRPower = friction * (y - x) + rx;
                bLPower = friction * (y - x) - rx;
                bRPower = friction * (y + x) + rx;
            }
            else {
                //Regular Code
                lockAngle = orientation;
                friction = 1;
                fLPower = y + x - rx;
                fRPower = y - x + rx;
                bLPower = y - x - rx;
                bRPower = y + x + rx;
            }

            //ninety degree turn code
            if (gamepad1.dpad_left) {
                positionalRotationMode = true;
                if (90 > orientation && orientation >= 0) {
                    targetAngle = 90; //90
                }
                else if (180 > orientation && orientation >= 90) {
                    targetAngle = 180; //180
                }
                else if (270 > orientation && orientation >= 180) {
                    targetAngle = 270; //270
                }
                else if (360 > orientation && orientation >= 270) {
                    targetAngle = 360; //360
                }
            }
            else if (gamepad1.dpad_right) {
                positionalRotationMode = true;
                if (90 > orientation && orientation >= 0) {
                    targetAngle = 0; //0
                }
                else if (180 > orientation && orientation >= 90) {
                    targetAngle = 90; //90
                }
                else if (270 > orientation && orientation >= 180) {
                    targetAngle = 180; //180
                }
                else if (360 > orientation && orientation >= 270) {
                    targetAngle = 270; //270
                }
            }

            if (positionalRotationMode) {
                if (!((targetAngle + 5) > orientation && orientation > (targetAngle - 5))) {
                    fLPower = PIDControl(targetAngle, orientation);
                    fRPower = -PIDControl(targetAngle, orientation);
                    bLPower = PIDControl(targetAngle, orientation);
                    bRPower = -PIDControl(targetAngle, orientation);
                    x = 0; //To make sure denominator code works correctly
                    y = 0;
                }
                else {
                    positionalRotationMode = false;
                }
            }

            //Power train calculations and motor power implementation
            double denominator = Math.max(friction * (Math.abs(x) + Math.abs(y)) + Math.abs(rx), 1);
            fLPower /= denominator;
            fRPower /= denominator;
            bLPower /= denominator;
            bRPower /= denominator;

            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            telemetry.addData("PID", orientation);
            telemetry.update();
        }
    }
    //PID Calculation Code (input = target) (output = power required)
    public double PIDControl(double target, double state) {
        previousError = error;
        error = target - state;
        integralSum += error * timer.seconds();
        derivative = (error - previousError) / timer.seconds();

        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
