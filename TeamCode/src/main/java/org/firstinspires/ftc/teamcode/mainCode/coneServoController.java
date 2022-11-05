package org.firstinspires.ftc.teamcode.mainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//cone upright arm controls (be sure to start bot with servo fully upright)
public class coneServoController extends LinearOpMode {

    private Servo coneServo;
    private boolean armDown, spamLock;

    @Override
    public void runOpMode() throws InterruptedException {
        //Empty space?
    }
    public void main() {
        coneServo = hardwareMap.get(Servo.class, "coneServo");
        coneServo.scaleRange(0.1, 0.425);
        coneServo.setDirection(Servo.Direction.REVERSE);

        if (gamepad2.b && !spamLock) {
            toggle();
            spamLock = true;
        }
        else if (!gamepad2.b) {
            spamLock = false;
        }
    }
    public void toggle() {
        armDown = !armDown;

        if (armDown) {
            coneServo.setPosition(1);
        }
        else {
            coneServo.setPosition(0);
        }
    }
}
