package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled

//Bot locking mechanism that makes the bot harder to move (can be used to stop quickly/hold position)
//Does not work with turns or rotational force
public class botLock {

    private double v, v1, v2, v3; //Same as RR (fL, bL, bR, fR)
    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    private double xState = 0, yState = 0; //Ticks off position (Y is forward, X is right)
    private double xCoeff, yCoeff; //Power coefficients for wheel directions
    private double integralSumX = 0, derivative, xError = 0, previousError;
    private double integralSumY, yError = 0; //Only error and integralSum have to be split for X/Y direction in PID
    private ElapsedTime xTimer = new ElapsedTime(); //For x PID
    private ElapsedTime yTimer = new ElapsedTime(); //For y PID
    private double Kp = 0, Kd = 0, Ki = 0; //PID stuff

    public botLock(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightFront");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "leftRear");
    }

    public void resetLock() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xTimer.reset();
        yTimer.reset();
        integralSumX = 0;
        integralSumY = 0;
    } //Runs once when function is activated to clear previous data from system

    public double[] lockBot() {
        xState = (leftEncoder.getCurrentPosition()+rightEncoder.getCurrentPosition())/2;
        yState = frontEncoder.getCurrentPosition();

        xCoeff = PIDx(xState);
        yCoeff = PIDy(yState);

        double denominator = Math.max(Math.max(Math.abs(xCoeff),Math.abs(yCoeff)), 1);
        xCoeff /= denominator;
        yCoeff /= denominator;

        v = xCoeff + yCoeff;
        v1 = yCoeff - xCoeff;
        v2 = xCoeff + yCoeff;
        v3 = yCoeff - xCoeff;

        double[] lockArray = {v, v1, v2, v3};
        return lockArray; //Data is returned as an array of power coefficients
    } //Position tracking, data distribution, and power assignment

    public double PIDx(double state) {
        previousError = xError;
        xError = -state;
        integralSumX += xError * xTimer.seconds();
        derivative = (xError - previousError) / xTimer.seconds();

        xTimer.reset();
        double output = (xError * Kp) + (derivative * Kd) + (integralSumX * Ki);
        return output;
    } //For calculating PID on x plane (left/right)

    public double PIDy(double state) {
        previousError = yError;
        yError = -state;
        integralSumY += yError * yTimer.seconds();
        derivative = (yError - previousError) / yTimer.seconds();

        yTimer.reset();
        double output = (yError * Kp) + (derivative * Kd) + (integralSumY * Ki);
        return output;
    } //For calculating PID on y plane (forward/back)
}
