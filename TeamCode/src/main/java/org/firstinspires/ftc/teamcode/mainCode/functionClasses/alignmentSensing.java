package org.firstinspires.ftc.teamcode.mainCode.functionClasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class alignmentSensing {

    private DistanceSensor dSensor;

    //For PID
    private double previousError = 0, error = 0, integralSum, derivative;
    private double Kp = 0.0045, Kd = 0, Ki = 0; //Don't use Ki
    private double acceptableError = 15; //Ticks of acceptableError +/- in slide positions
    private ElapsedTime timer;

    public alignmentSensing(HardwareMap hardwareMap) {
        dSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public double alignment() {
        double state = dSensor.getDistance(DistanceUnit.MM) - 6; //minimum distance is 6mm
        double power = PIDControl(state);
        return power;
    }

    //Game pad 1 rumbles when dSensor < 50mm and dSensor > 10mm
    //Hold Y while Game pad 1 rumbles and attempting to access cone stack/cone pole
    public boolean rumbleSense1() {
        if (dSensor.getDistance(DistanceUnit.MM) < 50 && dSensor.getDistance(DistanceUnit.MM) > 10) {
            return true;
        }
        else {
            return false;
        }
    }

    //Game pad 2 rumbles when dSensor < 10mm
    //Game pad 2 rumbles when bot is in alignment
    public boolean rumbleSense2() {
        if (dSensor.getDistance(DistanceUnit.MM) < 10) {
            return true;
        }
        else {
            return false;
        }
    }

    public double PIDControl(double state) {
        previousError = error;
        error = 0 - state;
        integralSum += error * timer.seconds();
        derivative = (error - previousError) / timer.seconds();

        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        return output;
    }
}
