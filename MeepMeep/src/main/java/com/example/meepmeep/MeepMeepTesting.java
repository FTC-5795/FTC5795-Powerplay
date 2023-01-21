
package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//    RoadRunnerBotEntity Right1 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-25, 14.5, Math.toRadians(270)))
               //                 .splineToConstantHeading(new Vector2d(-24,14.5), Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-24, 14.5, Math.toRadians(270)), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(-17, 14.5, Math.toRadians(270)))
//                                .forward(4.5)
//                                .waitSeconds(4)
//                                .back(4.5)
//                                .lineToLinearHeading(new Pose2d(-55,10.5, Math.toRadians(180)))
//                                .waitSeconds(.5)
//                                .back(3)
//                                .lineToSplineHeading(new Pose2d(-20,14.5, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(-10,14.5, Math.toRadians(270)))
//                                .back(24)
//                                .build()

//                );
        RoadRunnerBotEntity Right2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(-25,14.5, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-50,14.5, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(-53, 10, Math.toRadians(180)))
                                .waitSeconds(.5)
                                .back(12)
                                .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
                                .back(24)
                                .build()

                );
//        RoadRunnerBotEntity Right3 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                                .splineToConstantHeading(new Vector2d(-24,12), Math.toRadians(0))
//                                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                                .waitSeconds(.5)
//                                .back(12)
//                                .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(-58,12, Math.toRadians(270)))
//                                .back(24)
//                                .build()
//
//                );
//        RoadRunnerBotEntity Left1 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                                .waitSeconds(.5)
//                                .back(7)
//                                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
//                                .back(24)
//                                .build()
//
//                );
//        RoadRunnerBotEntity Left2 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                                .waitSeconds(.5)
//                                .back(7)
//                                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(35,12, Math.toRadians(270)))
//                                .back(24)
//                                .build()
//
//                );
//        RoadRunnerBotEntity Left3 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
//                                .splineToConstantHeading(new Vector2d(24,12), Math.toRadians(180))
//                                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(24,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                                .splineToConstantHeading(new Vector2d(55, 12), Math.toRadians(0))
//                                .waitSeconds(.5)
//                                .back(7)
//                                .lineToSplineHeading(new Pose2d(20,12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
//                                .back(24)
//                                .build()
//                );
//        RoadRunnerBotEntity RightGroundBreaker = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .setDimensions(13.5, 17)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61.5, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-60,60, Math.toRadians(270)))

//                                .splineToConstantHeading(new Vector2d(-60,50), Math.toRadians(270))
//                                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(180))
//                                .waitSeconds(.5)
//                                .back(12)
//                                .lineToSplineHeading(new Pose2d(-20,12, Math.toRadians(270)))
//                                .waitSeconds(.5)
//                                .lineToSplineHeading(new Pose2d(-35,12, Math.toRadians(270)))
//                                .back(24)
//                                .build()
//
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(Left1)
//                .addEntity(Left2)
//                .addEntity(Left3)
//                .addEntity(Right1)
                .addEntity(Right2)
//                .addEntity(Right3)
//                .addEntity(RightGroundBreaker)
                .start();
    }
}
//  .addTemporalMarker(2, () -> {
//                                    Put code here to be run 2 seconds into the trajectory
//                                    Used for actions during with autonomous (like extending slides)