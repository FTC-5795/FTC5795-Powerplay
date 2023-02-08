package org.firstinspires.ftc.teamcode.otherCode.diagnostics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.cameraController;

@Autonomous
public class cameraControllerDiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        cameraController controller = new cameraController(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            int parkingSpace = controller.camera();
            telemetry.addData("Parking Space", parkingSpace);
            telemetry.update();
        }
    }
}
