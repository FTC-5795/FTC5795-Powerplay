package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Deluxe TeleOp, the 5-Star Premium Supreme Elite Version
Features included are moving (duh), drift (left bumper), and 90-degree locked rotation.
*/

@TeleOp

public class DeluxeTeleOp extends LinearOpMode {

    //component declarations
    private DcMotorEx fL, fR, bL, bR; //motor declarations
    private double fLPower, fRPower, bLPower, bRPower; //motor power coefficients

    //drivetrain variables
    private double y, x, rx; //for driving
    private double lockAngle, orientation, friction; //for drift physics
    private boolean positionalRotationMode; //for ninety degree turn code (boolean)
    private double targetAngle; //for ninety degree turn code (double)

    //PID variables
    private double integralSum, derivative, error, previousError = 0; //for PID control (dynamic)
    private static double Kp = 2.25; //Proportional Gain (for more power)
    private static double Kd = 0.5; //Derivative Gain (increase to prevent overshoot)
    private static double Ki = 0; //Integral Gain (steady state error)

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{

        //for imu initialization
        BNO055IMU imu;
        BNO055IMU.Parameters gyro = new BNO055IMU.Parameters();
        gyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //gyro.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        gyro.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro);

        //Servo class initializations
        //hSlideServoController hSlideServo = new hSlideServoController();
        coneServoController coneServo = new coneServoController();
        /* gripServoController gripServo = new gripServoController();
        vSlideMotorController vSlideMotor = new vSlideMotorController(); */

        //Motor assignment
        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            //Dynamic variables with precision code (sqrt) on y & x
            y = Math.sqrt(Math.abs(-gamepad1.left_stick_y));
            x = 1.1 * Math.sqrt(Math.abs(gamepad1.left_stick_x));
            rx = -gamepad1.right_stick_x;
            if (-gamepad1.left_stick_y<0) {
                y = -y;
            }
            if (gamepad1.left_stick_x<0) {
                x = -x;
            }

            //Orientation code 0-360 degrees (0 is forward)
            if (imu.getAngularOrientation().firstAngle >= 0) {
                orientation = imu.getAngularOrientation().firstAngle;
            }
            else if (imu.getAngularOrientation().firstAngle < 0) {
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

            //ninety degree turns (method)
            ninetyDegreeController();

            //function classes
            //hSlideServo.runOpMode();
            coneServo.main();
            /* gripServo.runOpMode();
            vSlideMotor.runOpMode(); */

            //Slow mode
            if (gamepad1.right_bumper) {
                fLPower /= 2;
                fRPower /= 2;
                bLPower /= 2;
                bRPower /= 2;
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

            //telemetry outputs
            telemetry.addData("orientation", orientation);
            telemetry.update();
        }
    }
    //PID Calculation Code (input = target) (output = power required)
    public double PIDControl(double target, double state) {
        previousError = error;
        error = target - state;
        integralSum += error * timer.seconds();
        derivative = (error - previousError) / timer.seconds();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    //Ninety degree turn code
    public void ninetyDegreeController() {
        if (gamepad1.dpad_left) {
            positionalRotationMode = true;

            if (orientation == 360) {
                orientation = 0;
            }

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

            if (orientation == 0) {
                orientation = 360;
            }

            if (90 >= orientation && orientation > 0) {
                targetAngle = 0; //0
            }
            else if (180 >= orientation && orientation > 90) {
                targetAngle = 90; //90
            }
            else if (270 >= orientation && orientation > 180) {
                targetAngle = 180; //180
            }
            else if (360 >= orientation && orientation > 270) {
                targetAngle = 270; //270
            }
        }

        positionTracking();
    }
    //Positional Rotation System (input targetAngle)
    public void positionTracking() {

        if (positionalRotationMode) {
            if (targetAngle - orientation > 180) {
                orientation += 360;
            }
            else if (targetAngle - orientation < -180) {
                orientation -= 360;
            }
            if (!((targetAngle + 0.1) > orientation && orientation > (targetAngle - 0.1))) { //precision control (currently +-0.1)
                fLPower = -PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                fRPower = PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                bLPower = -PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                bRPower = PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                x = 0; //for denominator code
                y = 0; //for denominator code
            }
            else {
                positionalRotationMode = false;
                timer.reset();
            }
        }
    }
}