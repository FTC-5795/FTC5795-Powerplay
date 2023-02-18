package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class verticalDiagnostic extends LinearOpMode {

    private DcMotorEx lowerVerticalMotor, upperVerticalMotor;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private double vSlidePower;
    boolean lower, upper;

    @Override
    public void runOpMode() throws InterruptedException {
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

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                vSlidePower = -0.25;
            }
            else if (gamepad1.dpad_up) {
                vSlidePower = 0.25;
            }
            else {
                vSlidePower = 0.00;
            }

            if (gamepad1.a) {
                lower = true;
                upper = false;
                telemetry.addLine("lower");
            }
            else if (gamepad1.b) {
                lower = true;
                upper = true;
                telemetry.addLine("both");
            }
            else if (gamepad1.y) {
                lower = false;
                upper = true;
                telemetry.addLine("upper");
            }

            if (lower) {
                lowerVerticalMotor.setPower(vSlidePower);
            }
            else {
                lowerVerticalMotor.setPower(0);
            }

            if (upper) {
                upperVerticalMotor.setPower(vSlidePower);
            }
            else {
                upperVerticalMotor.setPower(0);
            }

            telemetry.addData("sensorDistance in MM", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("sensorBlue", colorSensor.blue());
            telemetry.addData("sensorRed", colorSensor.red());
            telemetry.addData("sensorGreen", colorSensor.green());

            telemetry.addData("lowerPosition", lowerVerticalMotor.getCurrentPosition());
            telemetry.addData("upperPosition", upperVerticalMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}