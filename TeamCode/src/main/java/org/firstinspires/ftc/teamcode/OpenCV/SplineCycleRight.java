package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.cameraController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

public class SplineCycleRight extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private int slideLevel; //1-12 slide height adjustment in trajectories
    private static double poleDepth = 4; //inches forward/back bot will move when scoring
    private int stackHeight = 9; //begins with stack of 5
    private int parkLocation = 0; //1,2,3 from left to right

    enum State {
        INITIAL,
        REPEAT,
        TO_STACK,
        FROM_STACK,
        PRIMARY_PARK,
        SECONDARY_PARK,
        IDLE
    }

    State state = State.IDLE;

    Pose2d rightStartPose = new Pose2d(-30, 61.5, Math.toRadians(270)); //center of square
    Pose2d rightPrimaryPose = new Pose2d(-26, 7.25, Math.toRadians(-45));
    Pose2d rightStackPose = new Pose2d(-48,12, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);
        cameraController vision = new cameraController(hardwareMap);

        TrajectorySequence initialTraj = drive.trajectorySequenceBuilder(rightStartPose)
                .splineToConstantHeading(new Vector2d(-35, 55), Math.toRadians(270))
                .forward(27.5)
                .splineToSplineHeading(rightPrimaryPose, Math.toRadians(315))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                }) //From initial position to primary pose
                .build();

        TrajectorySequence toStackTraj = drive.trajectorySequenceBuilder(rightPrimaryPose)

                .build();

        TrajectorySequence fromStackTraj = drive.trajectorySequenceBuilder(rightStackPose)

                .build();

        TrajectorySequence primaryPark1 = drive.trajectorySequenceBuilder(rightPrimaryPose)

                .build();

        TrajectorySequence primaryPark2 = drive.trajectorySequenceBuilder(rightPrimaryPose)

                .build();

        TrajectorySequence primaryPark3 = drive.trajectorySequenceBuilder(rightPrimaryPose)

                .build();

        drive.setPoseEstimate(rightStartPose);
        grab.autoGrip(true);

        while(opModeInInit()) {
            parkLocation = vision.camera();
            telemetry.addLine("Initialization Complete");
            telemetry.addData("Parking Location", parkLocation);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        autoTimer.reset();
        state = State.INITIAL;
        drive.followTrajectorySequenceAsync(initialTraj);

        while (opModeIsActive()) {

            switch (state) {
                case INITIAL:
                    if (vision.camera() != 0) {
                        parkLocation = vision.camera();
                    }
                    if (!drive.isBusy()) {
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            grab.autoGrip(false);
                        }
                        state = State.REPEAT;
                    }
                    break;

                case REPEAT:
                    if (autoTimer.seconds() < 22) {
                        drive.followTrajectorySequenceAsync(toStackTraj);
                        state = State.TO_STACK;
                    }
                    else {
                        state = State.PRIMARY_PARK;
                    }
                    break;

                case TO_STACK:
                    if (drive.isBusy()) {
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            slide.autoVSlide(stackHeight-1);
                        }

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            grab.autoGrip(true);
                        }

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.2) {
                            drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);
                            drive.update();
                        }
                        drive.setMotorPowers(0,0,0,0);

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            slide.autoVSlide(10);
                        }
                        slideLevel = 10;

                        if (autoTimer.seconds() < 22) {
                            drive.followTrajectorySequenceAsync(fromStackTraj);
                            state = State.FROM_STACK;
                        }
                        else {
                            state = State.SECONDARY_PARK;
                        }
                    }
                    break;

                case FROM_STACK:
                    if (!drive.isBusy()) {
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            grab.autoGrip(false);
                        }
                        state = State.REPEAT;
                    }
                    break;
            }

            drive.update();
            slide.autoVSlide(slideLevel);

            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
