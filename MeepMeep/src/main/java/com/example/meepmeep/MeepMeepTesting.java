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

        RoadRunnerBotEntity BlueLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 61.5, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(24,13), Math.toRadians(180))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(25, 13), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 13), Math.toRadians(0))
                                .waitSeconds(.5)
                                .splineToConstantHeading(new Vector2d(40, 13), Math.toRadians(0))
                                .splineTo(new Vector2d(30,13), Math.toRadians(-90))
//                                .forward(22)
//                                .addTemporalMarker(2, () -> {
//                                    Put code here to be run 2 seconds into the trajectory
//                                    Used for actions during with autonomous (like extending slides)
//                                })
//                                .splineTo(new Vector2d(24,13), Math.toRadians(180))
//                                Vector2D has end coordinates
//                                The Math.toRadians is the final position that the robot will face
//                                The front of the robot will not always face forward but instead bend with the path

//                                .turn(Math.toRadians(90))
//                                .forward(48)
//                                .turn(Math.toRadians(90))
//                                .forward(10)
//                                .turn(Math.toRadians(-90))
//                                .waitSeconds(2)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .waitSeconds(2)
//                                .back(30)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueLeft)
                .start();
    }
}