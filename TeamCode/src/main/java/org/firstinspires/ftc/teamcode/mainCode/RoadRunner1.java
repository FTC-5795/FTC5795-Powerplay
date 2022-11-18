package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.hSlideServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;

@Autonomous

public class RoadRunner1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gripServoController grip = new gripServoController(hardwareMap);
        hSlideServoController hSlide = new hSlideServoController(hardwareMap);
        vSlideMotorController vSlide = new vSlideMotorController(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                .back(7)
                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
                .back(24)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);
    }
}
