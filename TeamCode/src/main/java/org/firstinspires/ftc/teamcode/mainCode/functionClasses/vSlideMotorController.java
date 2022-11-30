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

//Vertical slide motor controls (Using PID) (Be sure to start slides retracted)
//Spool diameter: 4.7 | PPR:
public class vSlideMotorController {

    private DcMotorEx vSlideMotor;
    private double vSlidePower;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private int targetLevel = 0;
    private boolean spamLockUP, spamLockDOWN;

    private double previousError = 0, error = 0, integralSum, derivative;
    private double Kp = 0.01, Kd = 0, Ki = 0; //Don't use Ki

    public vSlideMotorController(HardwareMap hardwareMap) {
        vSlideMotor = hardwareMap.get(DcMotorEx.class, "vSlideMotor");
        vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void vSlide (boolean dUP, boolean dDOWN, boolean grabPosition) {

        double state = -vSlideMotor.getCurrentPosition();

        //targetLevel assignment
        if (dUP && !spamLockUP) {
            targetLevel += 1;
            spamLockUP = true;
            timer.reset();
        }
        else if (!dUP) {
            spamLockUP = false;
        }
        if (dDOWN && !spamLockDOWN) {
            targetLevel -= 1;
            spamLockDOWN = true;
            timer.reset();
        }
        else if (!dDOWN) {
            spamLockDOWN = false;
        }

        if (grabPosition) {
            targetLevel = 1;
        } //used to automatically set slides to grab position

        if (targetLevel > 4) {
            targetLevel = 4;
        }
        else if (targetLevel < 0) {
            targetLevel = 0;
        }

        double target = targetLevelConversion(targetLevel);
        vSlidePower = PIDControl(target, state);
        vSlideMotor.setPower(vSlidePower);
    }
    public double PIDControl(double target, double state) {
        previousError = error;
        error = target - state;
        integralSum += error * timer.seconds();
        derivative = (error - previousError) / timer.seconds();

        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double targetLevelConversion(int targetLevel) { //converts 0-3 targetLevels to inches
        if (targetLevel == 0) {
            return 10; //safety +10
        } //ground level
        else if (targetLevel == 1) {
            return 1200;
        } //grab position level
        else if (targetLevel == 2) {
            return 1815;
        } //low pole level
        else if (targetLevel == 3) {
            return 3159;
        } //medium pole level
        else {
            return 4503;
        } //tall pole level
    }

    public void autoVSlide(int targetLevel) {

        //0 is ground, 1 is grab height, 2 is low pole, 3 is medium pole, 4 is tall pole

        double target = targetLevelConversion(targetLevel);
        timer2.reset();

        while (timer2.seconds() < 2) {
            double state = -vSlideMotor.getCurrentPosition();
            vSlidePower = PIDControl(target, state);
            vSlideMotor.setPower(vSlidePower);
        } //Runs for 2 seconds (time to allow slides to stabilize)
    }
}
