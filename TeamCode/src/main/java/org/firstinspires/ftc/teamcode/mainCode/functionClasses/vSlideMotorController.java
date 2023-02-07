package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Vertical slide motor controls (Using PID) (Start slides retracted with tension on both motors)
//Spool diameter: 2" | PPR: 134.4
//8 target levels (0,2,4,6,8 = grabPositions (down) | 1,3,5,7,9 = grabPositions (up) | 10-12 = scoring heights)
public class vSlideMotorController {

    private DcMotorEx lowerVerticalMotor, upperVerticalMotor;
    private double lowerVerticalPower, upperVerticalPower;
    private ElapsedTime lowerTimer = new ElapsedTime();
    private ElapsedTime upperTimer = new ElapsedTime();
    private int targetLevel = 0; //0 is the reset level
    private int previousTargetLevel = 0; //previous targetLevel
    private double target = 0; //target height of slides in ticks
    private boolean spamLockUP, spamLockDOWN, spamLockReset;
    private DistanceSensor sensor;

    //For PID
    private double previousError1 = 0, error1 = 0, integralSum1, derivative1;
    private double previousError2 = 0, error2 = 0, integralSum2, derivative2;
    private double Kp = 0.0045, Kd = 0, Ki = 0; //Don't use Ki
    private double acceptableError = 15; //Ticks of acceptableError +/- in slide positions
    private int grab = 0; //0 is neutral, 1 is grab, 2 is release

    public vSlideMotorController(HardwareMap hardwareMap) {
        lowerVerticalMotor = hardwareMap.get(DcMotorEx.class, "lowerVerticalMotor");
        lowerVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerVerticalMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperVerticalMotor = hardwareMap.get(DcMotorEx.class, "upperVerticalMotor");
        upperVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperVerticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
    }

    public void vSlide(boolean dUP, boolean dDOWN, int grabPosition, boolean slideReset, double tDOWN, double tUP) {

        double state1 = -lowerVerticalMotor.getCurrentPosition();
        double state2 = upperVerticalMotor.getCurrentPosition();
        previousTargetLevel = targetLevel;

        //targetLevel assignment

        if (grabPosition > 0) {
            targetLevel = grabPosition;
        }

        if (dUP && !spamLockUP) {
            if (targetLevel < 10) {
                targetLevel = 10;
            } else if (targetLevel < 12) {
                targetLevel += 1;
            }
            spamLockUP = true;
            lowerTimer.reset();
            upperTimer.reset();
        } else if (!dUP) {
            spamLockUP = false;
        }
        if (dDOWN && !spamLockDOWN) {
            if (targetLevel < 10 && targetLevel % 2 == 1) {
                targetLevel -= 1;
            } else if (targetLevel > 10) {
                targetLevel -= 1;
            }
            spamLockDOWN = true;
            lowerTimer.reset();
            upperTimer.reset();
        } else if (!dDOWN) {
            spamLockDOWN = false;
        }

        target = targetLevelConversion(targetLevel); //converts to encoder tick value
        positionalAdjustmentProfile(tDOWN, tUP); //manual adjustment of slides using triggers

        if (targetLevel < 10 && targetLevel % 2 == 0 && state1 - target < 250) {
            grab = 1;
        }
        else {
            grab = 0;
        }

        if (target < 0) {
            target = 0;
        }

        //Slide reset controls
        if (slideReset & !spamLockReset) {
            slideReset();
            spamLockReset = true;
        }
        else if (!slideReset) {
            spamLockReset = false;
        }

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
            return 40; //safety +40
        } //stack of 1 (down) | ground/reset level
        else if (targetLevel == 1) {
            return 400;
        } //stack of 1 (up)
        else if (targetLevel == 2) {
            return 200;
        } //stack of 2 (down)
        else if (targetLevel == 3) {
            return 500;
        } //stack of 2 (up)
        else if (targetLevel == 4) {
            return 340;
        } //stack of 3 (down)
        else if (targetLevel == 5) {
            return 630;
        } //stack of 3 (up)
        else if (targetLevel == 6) {
            return 475;
        } //stack of 4 (down)
        else if (targetLevel == 7) {
            return 800;
        } //stack of 4 (up)
        else if (targetLevel == 8) {
            return 620;
        } //stack of 5 (down)
        else if (targetLevel == 9) {
            return 975;
        } //stack of 5 (up)
        else if (targetLevel == 10) {
            return 1580;
        } //small pole
        else if (targetLevel == 11) {
            return 2580;
        } //medium pole
        else if (targetLevel == 12) {
            return 3505;
        } //tall pole

        return 0; //for resetting slides
    }

    //resets motor encoders using color/distance sensor
    public void slideReset() {

        double state1 = -lowerVerticalMotor.getCurrentPosition();
        double state2 = upperVerticalMotor.getCurrentPosition();

        while (state1 > 50 && state2 > 50) {
            state1 = -lowerVerticalMotor.getCurrentPosition();
            state2 = upperVerticalMotor.getCurrentPosition();
            lowerVerticalPower = PIDControl1(0, state1);
            upperVerticalPower = PIDControl2(0, state2);
            lowerVerticalMotor.setPower(lowerVerticalPower);
            upperVerticalMotor.setPower(upperVerticalPower);
        } //gets slides close to reset region

        while (sensor.getDistance(DistanceUnit.MM) < 20) {
            lowerVerticalMotor.setPower(-0.2);
            upperVerticalMotor.setPower(-0.2);
        } //fine tunes reset
        lowerVerticalMotor.setPower(0);
        upperVerticalMotor.setPower(0);

        lowerVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerVerticalMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperVerticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperVerticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void positionalAdjustmentProfile(double tDOWN, double tUP) {
        double adjustmentFactor = 150 * (tUP - tDOWN);
        target += adjustmentFactor;
    } //adjusts up to 150 encoder ticks up/down

    //Automatic slide function
    public void autoVSlide(int targetLevel) {

        //slide heights explained on info page
        double target = targetLevelConversion(targetLevel); //1-12
        double state1 = -lowerVerticalMotor.getCurrentPosition();
        double state2 = upperVerticalMotor.getCurrentPosition();
        lowerVerticalPower = PIDControl1(target, state1);
        upperVerticalPower = PIDControl2(target, state2);
        lowerVerticalMotor.setPower(lowerVerticalPower);
        upperVerticalMotor.setPower(upperVerticalPower);
    }

    //For back up use only
    public void vSlideBackUp(boolean dUP, boolean dDOWN, boolean grabPosition) {

        if (dUP) {
            lowerVerticalMotor.setPower(0.8);
            upperVerticalMotor.setPower(0.8);
        } else if (dDOWN) {
            lowerVerticalMotor.setPower(-0.8);
            upperVerticalMotor.setPower(-0.8);
        } else {
            lowerVerticalMotor.setPower(0);
            upperVerticalMotor.setPower(0);
        }
    }

    public int autoGrab() {
        return grab;
    }
}
