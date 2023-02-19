package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.cameraCode.cameraController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.gripServoController;
import org.firstinspires.ftc.teamcode.mainCode.functionClasses.vSlideMotorController;
import org.firstinspires.ftc.teamcode.otherCode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.otherCode.trajectorysequence.TrajectorySequence;

@Autonomous
public class SplineCycleLeft extends LinearOpMode {

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime sleepTimer = new ElapsedTime();
    private int slideLevel; //1-12 slide height adjustment in trajectories
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

    Pose2d leftStartPose = new Pose2d(39.4, 61.5, Math.toRadians(270)); //center of square
    Pose2d leftPrimaryPose = new Pose2d(28.5, 5, Math.toRadians(225));
    Pose2d leftStackPose = new Pose2d(60.35,14.5, Math.toRadians(0));
    Pose2d leftMiddlePose = new Pose2d(40,14,Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vSlideMotorController slide = new vSlideMotorController(hardwareMap);
        gripServoController grab = new gripServoController(hardwareMap);
        cameraController vision = new cameraController(hardwareMap);

        TrajectorySequence initialTraj = drive.trajectorySequenceBuilder(leftStartPose)
                .splineToConstantHeading(new Vector2d(33, 55), Math.toRadians(270))
                .forward(27.5)
                .splineToSplineHeading(leftPrimaryPose, Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                }) //From initial position to primary pose
                .build();

        TrajectorySequence toStackTraj = drive.trajectorySequenceBuilder(leftPrimaryPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = stackHeight;
                })
                .splineToSplineHeading(leftMiddlePose, Math.toRadians(0))
                .splineToLinearHeading(leftStackPose, Math.toRadians(0))
                .setReversed(false)
                .build();

        TrajectorySequence fromStackTraj = drive.trajectorySequenceBuilder(leftStackPose.minus(new Pose2d(-6,0,0)))
                .splineToLinearHeading(leftMiddlePose, Math.toRadians(180))
                .splineToSplineHeading(leftPrimaryPose, Math.toRadians(225))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    slideLevel = 12;
                })
                .build();

        TrajectorySequence primaryPark1 = drive.trajectorySequenceBuilder(leftPrimaryPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeLeft(16)
                .build(); //Park location 1 from primary pose

        TrajectorySequence primaryPark2 = drive.trajectorySequenceBuilder(leftPrimaryPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeRight(5)
                .build(); //Park location 2 from primary pose

        TrajectorySequence primaryPark3 = drive.trajectorySequenceBuilder(leftPrimaryPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeRight(30)
                .build(); //Park location 3 from primary pose

        TrajectorySequence secondaryPark1 = drive.trajectorySequenceBuilder(leftStackPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeLeft(16)
                .build(); //Park location 1 from stack pose

        TrajectorySequence secondaryPark2 = drive.trajectorySequenceBuilder(leftStackPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeRight(5)
                .build(); //Park location 2 from stack pose

        TrajectorySequence secondaryPark3 = drive.trajectorySequenceBuilder(leftStackPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideLevel = 0;
                })
                .splineToSplineHeading(new Pose2d(40,12,Math.toRadians(270)), Math.toRadians(180))
                .strafeRight(30)
                .build(); //Park location 3 from stack pose

        drive.setPoseEstimate(leftStartPose);
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
                            slide.autoVSlide(slideLevel);
                        } //time for pole to stabilize
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.25) {
                            grab.autoGrip(false);
                        }
                        state = State.REPEAT;
                    }
                    break;

                case REPEAT:
                    if (autoTimer.seconds() < 25 && stackHeight > 0) {
                        drive.followTrajectorySequenceAsync(toStackTraj);
                        state = State.TO_STACK;
                    }
                    else {
                        state = State.PRIMARY_PARK;
                    }
                    break;

                case TO_STACK:
                    if (!drive.isBusy()) {
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.65) {
                            slide.autoVSlide(stackHeight-1);
                        }

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            grab.autoGrip(true);
                        }

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.2) {
                            drive.setMotorPowers(-0.25,-0.25,-0.25,-0.25);
                            drive.update();
                        }
                        drive.setMotorPowers(0,0,0,0);

                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.5) {
                            slide.autoVSlide(10);
                        }
                        slideLevel = 10;
                        stackHeight -=2;

                        if (autoTimer.seconds() < 25) {
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
                        } //time for pole to stabilize
                        sleepTimer.reset();
                        while (sleepTimer.seconds() < 0.25) {
                            grab.autoGrip(false);
                        }
                        state = State.REPEAT;
                    }
                    break;

                case PRIMARY_PARK:
                    if (parkLocation == 1) {
                        drive.followTrajectorySequenceAsync(primaryPark1);
                    }
                    else if (parkLocation == 2) {
                        drive.followTrajectorySequenceAsync(primaryPark2);
                    }
                    else {
                        drive.followTrajectorySequenceAsync(primaryPark3);
                    }
                    state = State.IDLE;
                    break;

                case SECONDARY_PARK:
                    if (parkLocation == 1) {
                        drive.followTrajectorySequenceAsync(secondaryPark1);
                    }
                    else if (parkLocation == 2) {
                        drive.followTrajectorySequenceAsync(secondaryPark2);
                    }
                    else {
                        drive.followTrajectorySequenceAsync(secondaryPark3);
                    }
                    state = State.IDLE;
                    break;
            }

            drive.update();
            slide.autoVSlide(slideLevel);

            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}