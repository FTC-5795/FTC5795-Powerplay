package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.source.doctree.StartElementTree;

//Vertical slide motor controls (Includes motion profiling and PIDF) (Be sure to start slides retracted)
public class vSlideMotorController {

    private DcMotorEx vSlideMotor;
    private double vSlidePower, startX, startV;
    private double ticksPerInch = 537.7/6.283; //ticks per revolution divided by circumference of pulley
    private ElapsedTime timer = new ElapsedTime();
    private int targetLevel = 0;
    private boolean spamLock;

    private double previousError = 0, error = 0, integralSum, derivative;
    private double Kp = 1, Kd = 0, Ki = 0;

    public vSlideMotorController(HardwareMap hardwareMap) {
        vSlideMotor = hardwareMap.get(DcMotorEx.class, "vSlideMotor");
        vSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void vSlide (boolean dUP, boolean dDOWN) {

        double state = vSlideMotor.getCurrentPosition()/ticksPerInch;
        double velocity = vSlideMotor.getVelocity()/ticksPerInch;

        //targetLevel assignment
        if (dUP && !spamLock) {
            targetLevel += 1;
            spamLock = true;
            startX = state;
            startV = velocity;
            timer.reset();
        }
        else if (!dUP) {
            spamLock = false;
        }
        if (dDOWN && !spamLock) {
            targetLevel -= 1;
            spamLock = true;
            startX = state;
            startV = velocity;
            timer.reset();
        }
        else if (!dDOWN) {
            spamLock = false;
        }

        if (targetLevel > 3) {
            targetLevel = 3;
        }
        else if (targetLevel < 0) {
            targetLevel = 0;
        }

        double target = targetLevelConversion(targetLevel);
        vSlidePower = profileGenerator(state, target, startX, startV);
        vSlidePower = PIDControl(target, state); //TODO: Remove to enable profile generator
        vSlideMotor.setPower(vSlidePower);
    }

    public double profileGenerator(double state, double target, double startX, double startV) { //all in inches
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startX,startV,0),
                new MotionState(target,0,0),
                25,
                40,
                100
        ); //Calculates PIDF inputs based on positional data and constraints
        MotionState mState = profile.get(timer.seconds());

        PIDCoefficients coeffs = new PIDCoefficients(1,0,0);
        //https:docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#
        PIDFController controller = new PIDFController(coeffs,0,0,0);

        controller.setTargetPosition(mState.getX());
        controller.setTargetVelocity(mState.getV());
        controller.setTargetAcceleration(mState.getA());

        double correction = controller.update(state);
        return correction;
    }

    public double PIDControl(double target, double state) {
        previousError = error;
        error = target - state;
        integralSum += error * timer.seconds();
        derivative = (error - previousError) / timer.seconds();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        output = error;
        return output;
    }

    public double targetLevelConversion(int targetLevel) { //converts 0-3 targetLevels to inches
        if (targetLevel == 0) {
            return 0;
        }
        else if (targetLevel == 1) {
            return 14;
        }
        else if (targetLevel == 2) {
            return 24;
        }
        else {
            return 33.5;
        }
    }

    public void autoVSlide(int targetLevel) {

        double target = targetLevelConversion(targetLevel);
        double state = vSlideMotor.getCurrentPosition()/ticksPerInch;
        double startX = state;
        double startV = vSlideMotor.getVelocity()/ticksPerInch;
        timer.reset();

        while (!(state == target)) {
            state = vSlideMotor.getCurrentPosition()/ticksPerInch;
            vSlidePower = profileGenerator(state, target, startX, startV);
            vSlideMotor.setPower(vSlidePower);
        }
    }
}
