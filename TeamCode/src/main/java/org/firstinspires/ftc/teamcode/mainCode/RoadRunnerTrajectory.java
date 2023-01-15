package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */

//   SlideLevel.autoVSlide(0-12);
//        Grab.autoGrip(TRUE/FALSE);

@Autonomous(group = "drive")
public class RoadRunnerTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController SlideLevel = new vSlideMotorController(hardwareMap);
        gripServoController Grab = new gripServoController(hardwareMap);

        // Loading up trajectories before start

        drive.setPoseEstimate(new Pose2d(-35, 61.5, Math.toRadians(270)));

        TrajectorySequence Right1 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                .addDisplacementMarker(() -> {
//                    telemetry.addData("A", "B");
//                    telemetry.update();
//                })
                .splineToConstantHeading(new Vector2d(-24,13.5), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-18, 13.5, Math.toRadians(270)))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(-25,13.5, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-50,13.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-53, 10, Math.toRadians(180)))
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(-45, 13.5, Math.toRadians(180)))
                .back(3)
                .lineToSplineHeading(new Pose2d(-20,13.5, Math.toRadians(270)))
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(-10,13.5, Math.toRadians(270)))
                .back(24)
                .build();

//        TrajectorySequence Right2 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .back(12)
//                .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Right3 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                .waitSeconds(.5)
//                .back(12)
//                .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(-58,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left1 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left2 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(35,12, Math.toRadians(270)))
//                .back(24)
//                .build();
//
//        TrajectorySequence Left3 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                .waitSeconds(.5)
//                .back(7)
//                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                .waitSeconds(.5)
//                .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
//                .back(24)
//                .build();
        Grab.autoGrip(true);


        //after start has been pressed
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(Right1);
        sleep(2000);
        drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(Right1.end())

                                .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(2);
                                })

                               .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(12);
                                })

                                .addDisplacementMarker(() -> {
                                    Grab.autoGrip(false);
                                })

                                .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(10);
                                })
//

                                .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(9);
                                })

                                .addDisplacementMarker(() -> {
                                    Grab.autoGrip(true);
                                })

                                .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(10);
                                })
//
//                                .UNSTABLE_addTemporalMarkerOffset(4, () ->
//                                {SlideLevel.autoVSlide(10);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(5, () ->
//                                {SlideLevel.autoVSlide(9);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(5.5, () ->
//                                {Grab.autoGrip(true);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(5.75, () ->
//                                {SlideLevel.autoVSlide(10);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(6, () ->
//                                {SlideLevel.autoVSlide(12);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(7, () ->
//                                {Grab.autoGrip(false);})
//
//                                .UNSTABLE_addTemporalMarkerOffset(7.5, () ->
//                                {SlideLevel.autoVSlide(0);})

//                                .addDisplacementMarker(() -> {
//                                    telemetry.addData("I works", "true");
//                                    telemetry.update();
//                                })

                                .splineToConstantHeading(new Vector2d(-24,13.5), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-18, 13.5, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    SlideLevel.autoVSlide(12);
                                })
                                .waitSeconds(3)
                                .addDisplacementMarker(() -> {
                                    Grab.autoGrip(false);
                                })
                                .lineToSplineHeading(new Pose2d(-25,13.5, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-50,13.5, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-53, 10, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-45, 13.5, Math.toRadians(180)))
                                .back(3)
                                .lineToSplineHeading(new Pose2d(-20,13.5, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-10,13.5, Math.toRadians(270)))
                                .back(24)
                                .build()
        );

//        drive.followTrajectorySequence(Right2);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Right2.end())
//                        .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                        .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                        .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .back(12)
//                        .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//        drive.followTrajectorySequence(Right3);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Right3.end())
//                        .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                        .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                        .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .back(12)
//                        .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
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
//                        .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
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
//                        .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(35,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
//        drive.followTrajectorySequence(Left3);
//        sleep(2000);
//        drive.followTrajectorySequence(
//                drive.trajectorySequenceBuilder(Left2.end())
//                        .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                        .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                        .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                        .waitSeconds(.5)
//                        .back(7)
//                        .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                        .waitSeconds(.5)
//                        .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
//                        .back(24)
//                        .build()
//        );
    }
}
