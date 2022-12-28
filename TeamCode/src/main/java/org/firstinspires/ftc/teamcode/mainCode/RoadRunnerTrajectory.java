package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadRunnerTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Loading up trajectories before start

        TrajectorySequence Right1 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
                .waitSeconds(.5)
                .back(7)
                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(-14,12, Math.toRadians(270)))
                .back(24)
                .build();

//        TrajectorySequence Right2 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
//                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-33,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Right3 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
//                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-58,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left1 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left2 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(33,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left3 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
//                .back(24)
//                .build();

        //after start has been pressed
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(Right1);
        sleep(2000);
        drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(Right1.end())
                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-14,12, Math.toRadians(270)))
                                .back(24)
                                .build()
        );

//        drive.followTrajectorySequence(Right2);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Right2.end())
//                        .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
//                        .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                        .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(-33,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//        drive.followTrajectorySequence(Right3);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Right3.end())
//                        .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .splineToConstantHeading(new Vector2d(-25, 12), Math.toRadians(180))
//                        .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                        .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(-58,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//
//        drive.followTrajectorySequence(Left1);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Left1.end())
//                        .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//
//        drive.followTrajectorySequence(Left2);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Left2.end())
//                        .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(33,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//        drive.followTrajectorySequence(Left3);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Left2.end())
//                        .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
    }
}
