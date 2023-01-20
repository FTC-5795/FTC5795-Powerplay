package org.firstinspires.ftc.teamcode.mainCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

public class FiniteStateMachine extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private int slideLevel; //1-12
    private boolean grabAction; //true for grab, false for release
    private static double poleDepth = 4; //inches forward/back bot will move when scoring
    private int stackHeight = 9; //beings with stack of 5
    private double parkLocation; //1,2,3 from left to right

    enum State {
        INITIAL, //Scans signal beacon
        TRAJECTORY_1, //Bot goes to tall pole to score or loop
        REPEAT, //Bot checks remaining time to either score or park
        PARK, //Parks depending on results from Open CV
        IDLE
    }

    State state = State.IDLE;

    Pose2d startPose = new Pose2d(-35, 61.5, Math.toRadians(270));
    Pose2d rightPrimaryPose = new Pose2d(-24, 14.5, Math.toRadians(270));
    Pose2d rightStackPose = new Pose2d(-55,10.5, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence Trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                .splineToLinearHeading(rightPrimaryPose, 0)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .addDisplacementMarker(() -> {
                    grabAction = false;
                })
                .waitSeconds(1)
                .back(poleDepth)
                .addDisplacementMarker(() -> {
                    slideLevel = stackHeight;
                })
                .build();

        TrajectorySequence TrajectoryLoop = drive.trajectorySequenceBuilder(rightPrimaryPose)
                .lineToLinearHeading(rightStackPose)
                .addDisplacementMarker(() -> {
                    slideLevel = stackHeight-1;
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    grabAction = true;
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    slideLevel = stackHeight;
                    stackHeight -= 2;
                })
                .waitSeconds(1)
                .lineToLinearHeading(rightPrimaryPose)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    slideLevel = 12;
                })
                .forward(poleDepth)
                .addDisplacementMarker(() -> {
                    grabAction = false;
                })
                .waitSeconds(1)
                .back(poleDepth)
                .addDisplacementMarker(() -> {
                    slideLevel = stackHeight;
                })
                .build();

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

        waitForStart();
        if (isStopRequested()) return;

        autoTimer.reset();

        state = State.INITIAL;

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

            case TRAJECTORY_1:
                if (!drive.isBusy()) {
                    state = State.REPEAT;
                }
                break;

            case REPEAT:
                if (stackHeight < 1) {
                    state = State.PARK;
                }
                else if (autoTimer.seconds() < 22) {
                    state = State.TRAJECTORY_1;
                    drive.followTrajectorySequenceAsync(TrajectoryLoop);
                }
                else {
                    state = State.PARK;
                }
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
