package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.botLock;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.coneServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;


/* Deluxe TeleOp, the 5-Star Premium Supreme Elite Version
Features included are moving (duh), drift (left bumper), 90-degree locked rotation, and function classes.
Function classes include vertical slide, cone upright servo, and grip servo.
*/

@TeleOp

public class DeluxeTeleOp extends LinearOpMode {

    //component declarations
    private DcMotorEx fL, fR, bL, bR; //motor declarations
    private double fLPower, fRPower, bLPower, bRPower; //motor power coefficients

    //drivetrain variables
    private double y, x, rx; //for driving
    private double lockAngle, orientation; //for drift physics
    private boolean positionalRotationMode; //for ninety degree turn code (boolean)
    private double targetAngle; //for ninety degree turn code (double)
    private double acceptableError = 2; //degrees of error accepted in turn code
    private double practiceCoefficient = 1; //Adjust for practice
    private boolean spamLock1; //for botLock

    //PID variables
    private double integralSum = 0, derivative, error = 0, previousError; //for PID control (dynamic)
    private static double Kp = 2.5; //Proportional Gain (for more power)
    private static double Kd = 0; //Derivative Gain (increase to prevent overshoot)
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

        //Function class initializations
        coneServoController coneServo = new coneServoController(hardwareMap);
        gripServoController gripServo = new gripServoController(hardwareMap);
        vSlideMotorController vSlideMotor = new vSlideMotorController(hardwareMap);
        botLock botLock = new botLock(hardwareMap);

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

            //NFS Drift Code
            if (gamepad1.left_bumper) {
                y = -gamepad1.left_stick_y * Math.sin(Math.toRadians(lockAngle - orientation + 90));
                x = 1.1 * -gamepad1.left_stick_y * Math.cos(Math.toRadians(lockAngle - orientation + 90));
            }
            else {
                lockAngle = orientation;
            }

            //Power assignment
            fLPower = y + x - rx;
            fRPower = y - x + rx;
            bLPower = y - x - rx;
            bRPower = y + x + rx;

            //ninety degree turns (method)
            ninetyDegreeController();

            //function classes
            coneServo.cone(gamepad2.b);
            gripServo.grip(gamepad2.a);
            vSlideMotor.vSlide(gamepad2.dpad_up, gamepad2.dpad_down, gamepad1.x);

            //botLock class
            if (gamepad1.x && !spamLock1) {
                spamLock1 = true;
                botLock.resetLock();
            }
            else if (gamepad1.x && spamLock1) {
                double[] lockArray = botLock.lockBot();
                fLPower = lockArray[1];
                bLPower = lockArray[2];
                bRPower = lockArray[3];
                fRPower = lockArray[4];
            }
            else if (!gamepad1.x) {
                spamLock1 = false;
            }

            //Bot Drift Issues (Not NFS Drifting)
            fLPower *= 1; // 0%
            bLPower *= 0.984; // -1.6%
            bRPower *= 1.0425; // +4.25%
            fRPower *= 0.984; // -1.6%

            //Power train calculations and motor power implementation
            double denominator = Math.max(Math.max(Math.max(Math.abs(fLPower), Math.abs(fRPower)), Math.max(Math.abs(bLPower), Math.abs(bRPower))), 1);
            fLPower /= denominator;
            fRPower /= denominator;
            bLPower /= denominator;
            bRPower /= denominator;

            //Slow mode
            if (gamepad1.right_bumper) {
                fLPower /= 2;
                fRPower /= 2;
                bLPower /= 2;
                bRPower /= 2;
            }

            //For practicing
            fLPower *= practiceCoefficient;
            fRPower *= practiceCoefficient;
            bLPower *= practiceCoefficient;
            bRPower *= practiceCoefficient;

            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            //telemetry outputs
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("orientation", orientation);
            telemetry.addData("power", PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation)));
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

    //Ninety degree turn code
    public void ninetyDegreeController() {

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            positionalRotationMode = true;
        }

        if (acceptableError >= orientation || orientation >= 360-acceptableError) {
            if (gamepad1.dpad_right) {
                targetAngle = 270;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 90;
            }
        }
        else if (90-acceptableError > orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 0;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 90;
            }
        }
        else if (90+acceptableError >= orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 0;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 180;
            }
        }
        else if (180-acceptableError > orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 90;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 180;
            }
        }
        else if (180+acceptableError >= orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 90;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 270;
            }
        }
        else if (270-acceptableError > orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 180;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 270;
            }
        }
        else if (270+acceptableError >= orientation) {
            if (gamepad1.dpad_right) {
                targetAngle = 180;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 360;
            }
        }
        else {
            if (gamepad1.dpad_right) {
                targetAngle = 270;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 360;
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

            if (!((targetAngle + 0.25) > orientation && orientation > (targetAngle - 0.25))) { //precision control (currently +-0.25)
                fLPower = -PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                fRPower = PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                bLPower = -PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                bRPower = PIDControl(Math.toRadians(targetAngle), Math.toRadians(orientation));
                x = 0; //for denominator code
                y = 0; //for denominator code
            }
            else {
                positionalRotationMode = false;
                timer.reset(); //Prevents build up with integral PID control
                integralSum = 0; //Prevents overlap with integral PID control
            }
        }
    }
}