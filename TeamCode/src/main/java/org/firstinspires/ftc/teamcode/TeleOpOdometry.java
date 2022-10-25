package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
Currently implemented for three dead wheel odometry

Notes: -Start position set as (0,0) and 0 degrees (Unit Circle)
 */

@TeleOp

public class TeleOpOdometry extends LinearOpMode {

    private DcMotorEx fL, fR, bL, bR;
    private double fLPower, fRPower, bLPower, bRPower;
    private DcMotorEx encoderParaL, encoderParaR, encoderPerp;

    //Geometric constants of Odometry
    private static double width = ; //distance between encoderParaL and encoderParaR in inches
    private static double y_Offset = ; //distance between midpoint of encoderParaL & encoderParaR AND encoderPerp in inches
    private static double wheelDiameter = 1; //Odometry wheel diameter in inches
    private static int encoderTicks = 8192; //Ticks per revolution of REV Bore Encoder
    private static double inchesPerTick = (wheelDiameter * Math.PI)/(encoderTicks);

    //Positioning constants (counted in ticks not inches)
    private int currentParaL = 0;
    private int currentParaR = 0;
    private int currentPerp = 0;

    private int previousParaL = 0;
    private int previousParaR = 0;
    private int previousPerp = 0;

    //Starting and current positions/heading of bot (counted in inches and degrees)
    private double xPosition = 0;
    private double yPosition = 0;
    private double heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Calibrating...");
        telemetry.update();

        //Motor mapping and calibration
        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fL.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bL.setDirection(DcMotorSimple.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Shadowing motors with encoders (for porting purposes)
        encoderParaL = fL;
        encoderParaR = fR;
        encoderPerp = bL;

        resetEncoders();

        telemetry.addData("Status", "Calibration Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            fLPower = y + x - rx;
            fRPower = y - x + rx;
            bLPower = y - x - rx;
            bRPower = y + x + rx;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            fLPower /= denominator;
            fRPower /= denominator;
            bLPower /= denominator;
            bRPower /= denominator;

            fL.setPower(fLPower);
            fR.setPower(fRPower);
            bL.setPower(bLPower);
            bR.setPower(bRPower);

            odometry();

            telemetry.addData("xPosition", xPosition);
            telemetry.addData("yPosition", yPosition);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }

    public void resetEncoders() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Odometry code
    public void odometry() {
        previousParaL = currentParaL;
        previousParaR = currentParaR;
        previousPerp = currentPerp;

        //Negate following to reverse encoder direction
        currentParaL = encoderParaL.getCurrentPosition();
        currentParaR = -encoderParaR.getCurrentPosition();
        currentPerp = encoderPerp.getCurrentPosition();

        //change in encoders in ticks
        int deltaParaL = currentParaL - previousParaL; //change in ParaL
        int deltaParaR = currentParaR - previousParaR; //change in ParaR
        int deltaPerp = currentPerp - previousPerp; //change in Perp

        //change in position (relative to bot) in inches
        double deltaX = inchesPerTick * (deltaParaL + deltaParaR) / 2;
        double deltaY = inchesPerTick * (deltaPerp - (deltaParaR - deltaParaL) * (y_Offset / width));
        double deltaTheta = inchesPerTick * (deltaParaR - deltaParaL) / width;

        //position update (relative to field) in inches and degrees
        double theta = heading + Math.toDegrees(deltaTheta / 2); //theta used for trig calculations
        xPosition += deltaX * Math.cos(Math.toRadians(theta)) - deltaY * Math.sin(Math.toRadians(theta));
        yPosition += deltaX * Math.sin(Math.toRadians(theta)) + deltaY * Math.cos(Math.toRadians(theta));
        heading += deltaTheta;
    }
}
