package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

public class FiniteStateMachine extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();

    enum State {
        INITIAL, //Scans signal beacon
        TRAJECTORY_1, //Bot goes to tall pole to score
        REPEAT, //Bot checks remaining time to either score or park
        TRAJECTORY_LOOP, //Bot collects cone from stack and scores
        PARK, //Parks depending on results from Open CV
        IDLE
    }

    State state = State.IDLE;

    //Staring pose of robot
    Pose2d startPose = new Pose2d(-35, 61.5, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slideLevel = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-24,13.5), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slideLevel.autoVSlide(12);
                })
                .forward(2.5)
                .build();
    }
}
