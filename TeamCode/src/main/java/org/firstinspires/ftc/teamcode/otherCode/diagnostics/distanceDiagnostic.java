package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class distanceDiagnostic extends LinearOpMode {

    private DistanceSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        waitForStart();
        while (opModeIsActive()) {
            double distance = sensor.getDistance(DistanceUnit.MM);
            telemetry.addData("Distance:", distance);
            telemetry.update();
        }
    }
}
