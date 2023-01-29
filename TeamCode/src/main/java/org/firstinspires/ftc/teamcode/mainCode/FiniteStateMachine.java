package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

//Starts by using OpenCV in initial
//Trajectory1 sends bot to pole; TRAJECTORY_1 -> TRAJECTORY_SCORE -> rightPrimaryPose (Score Routine);
//REPEAT_1 -> REPEAT_2 -> Score Routine (if enough time)
//or REPEAT_1 -> Park at end

@Autonomous
public class FiniteStateMachine extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private int slideLevel; //1-12 slide height adjustment in trajectories
    private static double poleDepth = 2; //inches forward/back bot will move when scoring
    private int stackHeight = 9; //begins with stack of 5
    private double parkLocation = 1; //1,2,3 from left to right

    enum State {
        INITIAL, //Scans signal beacon
        TRAJECTORY_1, //Bot goes to tall pole
        TRAJECTORY_SCORE, //Bot
        REPEAT_1,
        REPEAT_A,
        REPEAT_2,
        REPEAT_B,
        PARK, //Parks depending on results from Open CV
        IDLE
    }

    State state = State.IDLE;

    Pose2d startPose = new Pose2d(-35, 61.5, Math.toRadians(270));
    Pose2d rightPrimaryPose = new Pose2d(-20, 12, Math.toRadians(270));
    Pose2d rightStackPose = new Pose2d(-55,10.5, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(rightPrimaryPose, 0)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .build(); //First Trajectory goes from start rightPrimaryPose (autoSlide)

        TrajectorySequence TrajectoryScoreReturn = drive.trajectorySequenceBuilder(Trajectory1.end())
                .back(poleDepth)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideLevel = stackHeight;
                })
                .build(); //moves back, drops slides

        TrajectorySequence Repeat1 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-55, 11), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    slideLevel = stackHeight-1;
                })
                .build(); //goes to cone stack

        TrajectorySequence Repeat2 = drive.trajectorySequenceBuilder(Repeat1.end())
                .back(12)
                .lineToSplineHeading(rightPrimaryPose)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .build(); //returns from cone stack

        TrajectorySequence Parking1 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-10,14.5, Math.toRadians(270)))
                .back(24)
                .build();

        TrajectorySequence Parking2 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
                .back(24)
                .build();

        TrajectorySequence Parking3 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-58,14.5, Math.toRadians(270)))
                .back(24)
                .build();

        grab.autoGrip(true);

        waitForStart();
        if (isStopRequested()) return;

        autoTimer.reset();
        slide.autoVSlide(1);
        state = State.INITIAL;

        while (opModeIsActive()) {

            switch (state) {
                case INITIAL:
                    //Open CV stuff goes here
                    //parkLocation = 1,2,3
                    //if (tag found 17, 18, 19) {
                    state = State.TRAJECTORY_1;
                    drive.followTrajectorySequenceAsync(Trajectory1);
                    // }
                    //else if (autoTimer.seconds() > 2) {
                    //parkLocation = 2;
                    //state = State.TRAJECTORY_1;
                    //drive.followTrajectorySequenceAsync(Trajectory1);
                    // } Override if can't find tag
                    break;

                case TRAJECTORY_1: //Directs code to Score Routine
                    if (!drive.isBusy()) {
                        state = State.TRAJECTORY_SCORE;
                    }
                    break;

                case TRAJECTORY_SCORE: //Drops cone, move back, goes to stack height
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        grab.autoGrip(false);
                    }
                    drive.followTrajectorySequenceAsync(TrajectoryScoreReturn);
                    state = State.REPEAT_A;
                    break;

                case REPEAT_A:
                    if (!drive.isBusy()) {
                        state = State.REPEAT_1;
                    }
                    break;

                case REPEAT_1:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(stackHeight); //confirms prior slide height statement
                    }
                    if (stackHeight < 1) {
                        state = State.PARK;
                    }
                    else if (autoTimer.seconds() < 22) {
                        state = State.REPEAT_B;
                        drive.followTrajectorySequenceAsync(Repeat1);
                    }
                    else {
                        state = State.PARK;
                    }
                    break;

                case REPEAT_B:
                    if (!drive.isBusy()) {
                        state = State.REPEAT_2;
                    }
                    break;

                case REPEAT_2:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(stackHeight-1); //confirms prior slide height statement
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        grab.autoGrip(true);
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(stackHeight); //confirms prior slide height statement
                    }
                    stackHeight -= 2;
                    drive.followTrajectorySequenceAsync(Repeat2);
                    state = State.TRAJECTORY_1;
                    break;

                case PARK:
                    if (parkLocation == 1) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking1);
                    }
                    else if (parkLocation == 2) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking2);
                    }
                    else if (parkLocation == 3) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking3);
                    }
                    break;
            }

            drive.update();
            slide.autoVSlide(slideLevel);
        }
    }
}
