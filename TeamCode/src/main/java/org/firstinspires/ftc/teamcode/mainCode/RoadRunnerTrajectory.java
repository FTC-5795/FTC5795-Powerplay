package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        TrajectorySequence BlueLeft1 = drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                .waitSeconds(.5)
                .back(7)
                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                .waitSeconds(.5)
                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
                .back(24)
                .build();

        waitForStart();

        if (isStopRequested()) return;



        drive.followTrajectorySequence(BlueLeft1);
        sleep(2000);
        drive.followTrajectorySequence(
                        drive.trajectorySequenceBuilder(BlueLeft1.end())
                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
                                .back(24)
                                .build()

        );
    }
}
