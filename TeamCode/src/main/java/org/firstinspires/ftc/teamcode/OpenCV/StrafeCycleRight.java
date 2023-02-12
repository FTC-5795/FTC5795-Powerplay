package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.cameraController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

//Starts by using OpenCV in initial
//Trajectory1 sends bot to pole; TRAJECTORY_1 -> TRAJECTORY_SCORE -> rightPrimaryPose (Score Routine);
//REPEAT_1 -> REPEAT_2 -> Score Routine (if enough time)
//or REPEAT_1 -> Park at end

@Disabled
@Autonomous
public class StrafeCycleRight extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private int slideLevel; //1-12 slide height adjustment in trajectories
    private static double poleDepth = 4; //inches forward/back bot will move when scoring
    private int stackHeight = 9; //begins with stack of 5
    private int parkLocation = 0; //1,2,3 from left to right

    enum State {
        INITIAL, //Scans signal beacon
        START_TRAJECTORY, //Bot goes to tall pole
        SCORE_RETURN, //Bot drops cone and returns to primary pose
        TO_STACK_TRAJECTORY, //Bot goes to cone stack
        SCORE_WAIT, //Checks for drive.busy
        FROM_STACK_TRAJECTORY, //Bot goes back to primary pose
        STACK_WAIT, //Checks for drive.busy
        PARK, //Parks from leftPrimaryPose depending on results from Open CV
        PARK_2, //Parks with preloaded cone
        IDLE
    }

    State state = State.IDLE;

    Pose2d rightStartPose = new Pose2d(-31, 61.5, Math.toRadians(270));
    Pose2d rightPrimaryPose = new Pose2d(-20, 13, Math.toRadians(270));
    Pose2d rightStackPose = new Pose2d(-56,9, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);
        cameraController vision = new cameraController(hardwareMap);

        drive.setPoseEstimate(rightStartPose);

        TrajectorySequence initialTrajectory = drive.trajectorySequenceBuilder(rightStartPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    grab.autoGrip(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    slideLevel = 2;
                })
                .strafeRight(1.5)
                .build();

        TrajectorySequence startTrajectory = drive.trajectorySequenceBuilder(initialTrajectory.end())
                .strafeRight(2.5)
                .forward(48.5)
                .strafeLeft(15)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .build(); //First Trajectory goes from start rightPrimaryPose (autoSlide)

        TrajectorySequence scoreReturn = drive.trajectorySequenceBuilder(startTrajectory.end())
                .back(poleDepth)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideLevel = stackHeight;
                })
                .build(); //moves back, drops slides

        TrajectorySequence toStackTrajectory = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-30,12, Math.toRadians(180)))
                .splineToLinearHeading(rightStackPose, Math.toRadians(180))
                .build(); //goes to cone stack

        TrajectorySequence fromStackTrajectory = drive.trajectorySequenceBuilder(toStackTrajectory.end())
                .back(12)
                .lineToSplineHeading(rightPrimaryPose)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .build(); //returns from cone stack

        TrajectorySequence Parking1 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-10,14.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        TrajectorySequence preParking1 = drive.trajectorySequenceBuilder(rightStackPose)
                .back(30)
                .lineToSplineHeading(new Pose2d(-10,14.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        TrajectorySequence Parking2 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        TrajectorySequence preParking2 = drive.trajectorySequenceBuilder(rightStackPose)
                .back(15)
                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        TrajectorySequence Parking3 = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToSplineHeading(new Pose2d(-56.5,14.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        TrajectorySequence preParking3 = drive.trajectorySequenceBuilder(rightStackPose)
                .lineToSplineHeading(new Pose2d(-56.5,14.5, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.675, () -> {
                    slideLevel = 0;
                })
                .build();

        grab.autoGrip(true);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        autoTimer.reset();
        state = State.INITIAL;
        drive.followTrajectorySequenceAsync(initialTrajectory);

        while (opModeIsActive()) {

            switch (state) {
                case INITIAL:
                    if (!drive.isBusy()) {
                        while (parkLocation == 0 && autoTimer.seconds() < 5) {
                            parkLocation = vision.camera();
                            grab.autoGrip(true);
                            slide.autoVSlide(1);
                        }
                        state = State.START_TRAJECTORY;
                        drive.followTrajectorySequenceAsync(startTrajectory);
                    }
                    break;

                case START_TRAJECTORY: //Directs code to Score Routine
                    if (!drive.isBusy()) {
                        state = State.SCORE_RETURN;
                    }
                    break;

                case SCORE_RETURN: //Drops cone, move back, goes to stack height
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 0.7) {
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        grab.autoGrip(false);
                    }
                    drive.followTrajectorySequenceAsync(scoreReturn);
                    state = State.SCORE_WAIT;
                    break;

                case SCORE_WAIT:
                    if (!drive.isBusy()) {
                        state = State.TO_STACK_TRAJECTORY;
                    }
                    break;

                case TO_STACK_TRAJECTORY:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(stackHeight); //confirms prior slide height statement
                    }
                    if (stackHeight < 1) {
                        state = State.PARK;
                    }
                    else if (autoTimer.seconds() < 20) {
                        state = State.STACK_WAIT;
                        drive.followTrajectorySequenceAsync(toStackTrajectory);
                    }
                    else {
                        state = State.PARK;
                    }
                    break;

                case STACK_WAIT:
                    if (!drive.isBusy()) {
                        state = State.FROM_STACK_TRAJECTORY;
                    }
                    break;

                case FROM_STACK_TRAJECTORY:
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(stackHeight-1); //confirms prior slide height statement
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        grab.autoGrip(true);
                    }
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 0.2) {
                        drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);
                        drive.update();
                    }
                    drive.setMotorPowers(0,0,0,0);
                    sleepTimer.reset();
                    while (sleepTimer.seconds() < 1) {
                        slide.autoVSlide(10);
                    }
                    slideLevel = 10;
                    stackHeight -= 2;
                    if (autoTimer.seconds() < 16) {
                        drive.followTrajectorySequenceAsync(fromStackTrajectory);
                        state = StrafeCycleRight.State.START_TRAJECTORY;
                    }
                    else {
                        state = StrafeCycleRight.State.PARK_2;
                    }
                    break;

                case PARK:
                    if (parkLocation == 1) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking1);
                    }
                    else if (parkLocation == 2 || parkLocation == 0) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking2);
                    }
                    else if (parkLocation == 3) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(Parking3);
                    }
                    break;

                case PARK_2:
                    if (parkLocation == 1) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(preParking1);
                    }
                    else if (parkLocation == 2 || parkLocation == 0) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(preParking2);
                    }
                    else if (parkLocation == 3) {
                        state = State.IDLE;
                        drive.followTrajectorySequenceAsync(preParking3);
                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.update();

            drive.update();
            slide.autoVSlide(slideLevel);
        }
    }
}
