package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class WheelDiagnostic extends LinearOpMode {

    private double fLCoeff = 1, fRCoeff = 1, bLCoeff = 1, bRCoeff = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.setMotorPowers(fLCoeff*0.5, bLCoeff*0.5, bRCoeff*0.5, fRCoeff*0.5);

                sleep(3000);

                drive.setMotorPowers(0,0,0,0);
            }
            else if (gamepad1.b) {
                drive.setMotorPowers(fLCoeff*-0.5, bLCoeff*-0.5, bRCoeff*-0.5, fRCoeff*-0.5);

                sleep(3000);
                drive.setMotorPowers(0,0,0,0);
            }
        }
    }
}
