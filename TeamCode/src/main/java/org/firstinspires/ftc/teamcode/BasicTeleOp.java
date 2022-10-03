package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="BasicTeleOp", group = "a")
public class BasicTeleOp extends LinearOpMode {

    private DcMotorEx fL, fR, bL,bR;
    private double flPower, frPower, blPower, brPower;

    public enum state {
        TURN, NONE;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU imu;
        BNO055IMU.Parameters gyro = new BNO055IMU.Parameters();
        gyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //gyro.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample op mode
        gyro.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(gyro);

        telemetry.addData("Mode", "calibrating...");

        fL = hardwareMap.get(DcMotorEx.class, "leftFront");
        fR = hardwareMap.get(DcMotorEx.class, "rightFront");
        bL = hardwareMap.get(DcMotorEx.class, "leftRear");
        bR = hardwareMap.get(DcMotorEx.class, "rightRear");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            boolean leftTurn = gamepad1.dpad_left;
            boolean rightTurn = gamepad1.dpad_right;
            telemetry.addData("turn", rx);
            telemetry.addData("Angle", getAngle(imu));
            telemetry.addData("Y", y);
            telemetry.addData("X", x);

            flPower = y + x - rx;
            frPower = y - x + rx;
            blPower = y - x - rx;
            brPower = y + x + rx;

            boolean startTurn = false;

            if(leftTurn || rightTurn) {
                startTurn = true;
            }

            if(startTurn) {
                double prevAngle = getAngle(imu);
                double newAngle = 0;

                /*
                if(rightTurn && prevAngle > 90) {
                    double angleCorrection = Math.abs(prevAngle - 90) * 2;
                    newAngle = ((prevAngle * -1) + angleCorrection) - 90;
                } else if (leftTurn && prevAngle < -90){
                    double angleCorrection = Math.abs(prevAngle + 90) * 2;
                    newAngle = ((prevAngle * -1) + angleCorrection) + 90;
                } else if (rightTurn && prevAngle < 90) {
                    newAngle = prevAngle + 90;
                } else if (leftTurn && prevAngle > -90) {
                    newAngle = prevAngle - 90;
                } */

                if (prevAngle > 180) {
                    prevAngle -= 180;
                }

                if (prevAngle < -180) {
                    prevAngle += 180;
                }

                if (rightTurn) {
                    newAngle = prevAngle + 90;
                } else {
                    newAngle = prevAngle - 90;
                }

                    telemetry.addData("Prev Angle", prevAngle);
                    telemetry.addData("New Angle",newAngle);

                flPower = -1;
                frPower = 1;
                blPower = -1;
                brPower = 1;

                double desiredAngle;

                switch (state) {
                    case NONE:
                        if (leftTurn) {
                            prevAngle = getAngle(imu);
                            desiredAngle = prevAngle - 90;
                            state = state.TURN;
                        }
                        if (rightTurn) {
                            prevAngle = getAngle(imu);
                            desiredAngle = prevAngle + 90;
                            state = state.TURN;
                        }
                        break;

                    case TURN:

                        break;

                }



                    /*while(getAngle(imu) != newAngle) {
                        flPower = -1;
                        frPower = 1;
                        blPower = -1;
                        brPower = 1;
                    } */
                startTurn = false;
            }

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            flPower /= denominator;
//            frPower /= denominator;
//            blPower /= denominator;
//            brPower /= denominator;
//
            fL.setPower(flPower);
            fR.setPower(frPower);
            bL.setPower(blPower);
            bR.setPower(brPower);

            DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

            double flyPower = -1 * gamepad1.right_trigger;

            intake.setPower(flyPower);
            telemetry.update();
        }


    }

    private double getAngle(BNO055IMU imu) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

}