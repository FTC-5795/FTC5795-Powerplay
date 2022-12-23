package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Vertical slide motor controls (Using PID) (Start slides retracted with tension on both motors)
//Spool diameter: 2" | PPR: 134.4
public class vSlideMotorController {

    private DcMotorEx lowerVerticalMotor, upperVerticalMotor;
    private double lowerVerticalPower, upperVerticalPower;
    private ElapsedTime lowerTimer = new ElapsedTime();
    private ElapsedTime upperTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();
    private int targetLevel = 0;
    private boolean spamLockUP, spamLockDOWN;

    private double previousError1 = 0, error1 = 0, integralSum1, derivative1;
    private double previousError2 = 0, error2 = 0, integralSum2, derivative2;
    private double Kp = 0.0045, Kd = 0, Ki = 0; //Don't use Ki
    private double acceptableError = 20; //Ticks of acceptableError +/- in slide positions

    public vSlideMotorController(HardwareMap hardwareMap) {
        lowerVerticalMotor = hardwareMap.get(DcMotorEx.class, "lowerVerticalMotor");
        lowerVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerVerticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        upperVerticalMotor = hardwareMap.get(DcMotorEx.class, "upperVerticalMotor");
        upperVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperVerticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void vSlide (boolean dUP, boolean dDOWN, boolean grabPosition) {

        double state1 = lowerVerticalMotor.getCurrentPosition();
        double state2 = upperVerticalMotor.getCurrentPosition();

        //targetLevel assignment
        if (dUP && !spamLockUP) {
            targetLevel += 1;
            spamLockUP = true;
            lowerTimer.reset();
            upperTimer.reset();
        }
        else if (!dUP) {
            spamLockUP = false;
        }
        if (dDOWN && !spamLockDOWN) {
            targetLevel -= 1;
            spamLockDOWN = true;
            lowerTimer.reset();
            upperTimer.reset();
        }
        else if (!dDOWN) {
            spamLockDOWN = false;
        }

        //used to automatically set slides to grab position
        if (grabPosition) {
            targetLevel = 1;
        }

        if (targetLevel > 4) {
            targetLevel = 4;
        }
        else if (targetLevel < 0) {
            targetLevel = 0;
        }

        double target = targetLevelConversion(targetLevel);
        lowerVerticalPower = PIDControl1(target, state1);
        upperVerticalPower = PIDControl2(target, state2);
        lowerVerticalMotor.setPower(lowerVerticalPower);
        upperVerticalMotor.setPower(upperVerticalPower);
    }

    //PID for primary (lower) slide motor
    public double PIDControl1(double target, double state) {
        previousError1 = error1;
        error1 = target - state;
        integralSum1 += error1 * lowerTimer.seconds();
        derivative1 = (error1 - previousError1) / lowerTimer.seconds();

        lowerTimer.reset();
        double output = (error1 * Kp) + (derivative1 * Kd) + (integralSum1 * Ki);


        if (error1 < acceptableError && error1 > -acceptableError) {
            output = 0; //allowing error saves battery power
        }


        return output;
    }
    //PID for secondary (upper) slide motor
    public double PIDControl2(double target, double state) {
        previousError2 = error2;
        error2 = target - state;
        integralSum2 += error2 * upperTimer.seconds();
        derivative2 = (error2 - previousError2) / upperTimer.seconds();

        upperTimer.reset();
        double output = (error1 * Kp) + (derivative1 * Kd) + (integralSum1 * Ki);


        if (error1 < acceptableError && error1 > -acceptableError) {
            output = 0; //allowing error saves battery power
        }


        return output;
    }
    public double targetLevelConversion(int targetLevel) { //converts 0-3 targetLevels to inches
        if (targetLevel == 0) {
            return 35; //safety +35
        } //ground level
        else if (targetLevel == 1) {
            return 350;
        } //grab position level
        else if (targetLevel == 2) {
            return 1650;
        } //low pole level
        else if (targetLevel == 3) {
            return 2650;
        } //medium pole level
        else {
            return 3550;
        } //tall pole level
    }

    public void autoVSlide(int targetLevel) {

        //0 is ground, 1 is grab height, 2 is low pole, 3 is medium pole, 4 is tall pole

        double target = targetLevelConversion(targetLevel);
        autoTimer.reset();

        while (autoTimer.seconds() < 2) {
            double state1 = lowerVerticalMotor.getCurrentPosition();
            double state2 = upperVerticalMotor.getCurrentPosition();
            lowerVerticalPower = PIDControl1(target, state1);
            upperVerticalPower = PIDControl2(target, state2);
            lowerVerticalMotor.setPower(lowerVerticalPower);
            upperVerticalMotor.setPower(upperVerticalPower);
        } //Runs for 2 seconds (time to allow slides to stabilize)
    }

    //For emergency use only
    public void vSlideBackUp (boolean dUP, boolean dDOWN, boolean grabPosition) {

        if (dUP) {
            lowerVerticalMotor.setPower(0.32);
            upperVerticalMotor.setPower(0.32);
        }
        else if (dDOWN) {
            lowerVerticalMotor.setPower(-0.32);
            upperVerticalMotor.setPower(-0.32);
        }
        else {
            lowerVerticalMotor.setPower(0);
            upperVerticalMotor.setPower(0);
        }
    }
}
