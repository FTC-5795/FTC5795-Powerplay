package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

public class BotTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d rightStartPose = new Pose2d(-31, 61.5, Math.toRadians(270));
        Pose2d rightPrimaryPose = new Pose2d(-30, 5, Math.toRadians(315));
        Pose2d rightStackPose = new Pose2d(-56,9, Math.toRadians(180));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(rightStartPose)
                .splineToSplineHeading(new Pose2d(-30,5,Math.toRadians(315)), Math.toRadians(315))
                .build();

        drive.followTrajectorySequence(startTraj);
    }
}
