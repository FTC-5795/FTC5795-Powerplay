package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity GBBlueLeft1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                                .strafeRight(23)
                                .waitSeconds(.5)
                                .forward(80)
                                .splineToConstantHeading(new Vector2d(0,-37), Math.toRadians(180))
                                .waitSeconds(1)
                                .strafeLeft(10)
                                .back(25)
                                .lineToSplineHeading(new Pose2d(18,12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .forward(42)
                                .waitSeconds(1)
                                .back(35)
                                .lineToSplineHeading(new Pose2d(24, 10, Math.toRadians(-90)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(8, 28, Math.toRadians(-270)))
                                .forward(5)
                                .waitSeconds(1)
                                .build()

                );

        RoadRunnerBotEntity BlueLeftCycle2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(35, 22 ), Math.toRadians(180))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity BlueLeftCycle1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(12, 22 ), Math.toRadians(360))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity BlueLeftCycle3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(25, 12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .back(7)
                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(35, 22 ), Math.toRadians(180))
                                .back(13)
                                .strafeLeft(25)
                                .build()
                );
        RoadRunnerBotEntity BlueRightCycle2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-35,22), Math.toRadians(360))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity BlueRightCycle3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-35,22), Math.toRadians(360))
                                .back(13)
                                .strafeRight(25)
                                .build()
                );
        RoadRunnerBotEntity BlueRightCycle1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-10,22), Math.toRadians(900))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity RedLeftCycle2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-24,-12), Math.toRadians(720))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-25, -12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-35, -22), Math.toRadians(360))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity RedLeftCycle1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-24,-12), Math.toRadians(720))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-25, -12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-10, -22), Math.toRadians(900))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity RedLeftCycle3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-24,-12), Math.toRadians(720))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-25, -12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-50,-12, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(-35, -22), Math.toRadians(360))
                                .back(13)
                                .strafeLeft(25)
                                .build()
                );

        RoadRunnerBotEntity RedRightCycle2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(24,-12), Math.toRadians(910))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(900))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity RedRightCycle1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(24,-12), Math.toRadians(910))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(10, -22), Math.toRadians(0))
                                .back(13)
                                .build()
                );
        RoadRunnerBotEntity RedRightCycle3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -61.5, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(24,-12), Math.toRadians(910))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(50, -12, Math.toRadians(0)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(90)))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(35, -22), Math.toRadians(900))
                                .back(13)
                                .strafeRight(25)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(GBBlueLeft1)
                .addEntity(BlueLeftCycle1)
                .addEntity(BlueLeftCycle2)
                .addEntity(BlueLeftCycle3)
                .addEntity(BlueRightCycle1)
                .addEntity(BlueRightCycle2)
                .addEntity(BlueRightCycle3)
                .addEntity(RedLeftCycle2)
                .addEntity(RedLeftCycle1)
                .addEntity(RedLeftCycle3)
                .addEntity(RedRightCycle2)
                .addEntity(RedRightCycle1)
                .addEntity(RedRightCycle3)
                .start();
    }
}
//  .addTemporalMarker(2, () -> {
//                                    Put code here to be run 2 seconds into the trajectory
//                                    Used for actions during with autonomous (like extending slides)