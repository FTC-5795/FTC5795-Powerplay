package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


//For normal servos
@TeleOp
public class Diagnostic2 extends LinearOpMode {

    private Servo gripServo, hSlideServo, coneServo;
    private double target;
    private double select = 1;
    private boolean spamLock, spamLock2;

    @Override
    public void runOpMode() throws InterruptedException {

        gripServo = hardwareMap.get(Servo.class, "gripServo");
        hSlideServo = hardwareMap.get(Servo.class, "hSlideServo");
        coneServo = hardwareMap.get(Servo.class, "coneServo");

        hSlideServo.scaleRange(0.1,0.38);
        coneServo.scaleRange(0,1);
        coneServo.setDirection(Servo.Direction.REVERSE);
        coneServo.setPosition(0.5);

       waitForStart();

       while (opModeIsActive()) {
           if (gamepad1.x) {
               select = 1;
           }
           else if (gamepad1.y) {
               select = 2;
           }
           else if (gamepad1.b) {
               select = 3;
           }

           if (select == 1) {
               target = hSlideServo.getPosition();
               hSlideServo.setPosition(position(hSlideServo.getPosition()));
               telemetry.addData("hSlideServo target", target);
               telemetry.addData("hSlideServo position", hSlideServo.getPosition());
           }
           else if (select == 2) {
               target = gripServo.getPosition();
               gripServo.setPosition(position(gripServo.getPosition()));
               telemetry.addData("gripServo target", target);
               telemetry.addData("gripServo position", gripServo.getPosition());
           }
           else if (select == 3) {
               target = coneServo.getPosition();
               coneServo.setPosition(position(coneServo.getPosition()));
               telemetry.addData("coneServo target", target);
               telemetry.addData("coneServo position", coneServo.getPosition());
           }
           telemetry.update();
       }
    }
    public double position(double state) {
        if (gamepad1.dpad_up && !spamLock) {
            target = state + 0.05;
            spamLock = true;
        }
        else if (!gamepad1.dpad_up) {
            spamLock = false;
        }

        if (gamepad1.dpad_down && !spamLock2) {
            target = state - 0.05;
            spamLock2 = true;
        }
        else if (!gamepad1.dpad_down) {
            spamLock2 = false;
        }
        return target;
    }
}
