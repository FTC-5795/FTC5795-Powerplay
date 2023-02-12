package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class BotTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d rightStartPose = new Pose2d(-35, 61.5, Math.toRadians(270));
        Pose2d rightPrimaryPose = new Pose2d(-26, 7.25, Math.toRadians(-45));
        Pose2d rightStackPose = new Pose2d(-55,10.5, Math.toRadians(180));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(rightStartPose)
                .forward(34)
                .splineToSplineHeading(rightPrimaryPose, Math.toRadians(315))
                .build();

        drive.setPoseEstimate(rightStartPose);

        waitForStart();

        drive.followTrajectorySequence(startTraj);
    }
}
