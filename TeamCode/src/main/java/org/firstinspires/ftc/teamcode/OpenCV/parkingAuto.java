package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.cameraController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

@Autonomous
public class parkingAuto extends LinearOpMode {

    double parkingSpace = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraController vision = new cameraController(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence initial = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(3)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .strafeLeft(24.5)
                .forward(30)
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .strafeRight(2.5)
                .forward(30)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .strafeRight(2.5)
                .strafeRight(46)
                .forward(30)
                .build();

        waitForStart();
        drive.followTrajectorySequence(initial);
        while (opModeIsActive()) {
            while (parkingSpace == 0) {
                parkingSpace = vision.camera();
            }
            if (parkingSpace == 1) {
                drive.followTrajectorySequence(left);
            }
            else if (parkingSpace == 2) {
                drive.followTrajectorySequence(middle);
            }
            else if (parkingSpace == 3) {
                drive.followTrajectorySequence(right);
            }
        }

    }
}
