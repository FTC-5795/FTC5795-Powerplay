package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepEngine3 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double poleDepth = 2;

        Pose2d rightStartPose = new Pose2d(-30, 61.5, Math.toRadians(270)); //center of square
        Pose2d rightPrimaryPose = new Pose2d(-26, 7.25, Math.toRadians(315));
        Pose2d rightStackPose = new Pose2d(-48,12, Math.toRadians(180));
        Pose2d rightMiddlePose = new Pose2d(-44,12,Math.toRadians(180));


        Pose2d leftStartPose = new Pose2d(35, 61.5, Math.toRadians(270));
        Pose2d leftPrimaryPose = new Pose2d(20, 12, Math.toRadians(270));
        Pose2d leftStackPose = new Pose2d(55,10.5, Math.toRadians(0));

        RoadRunnerBotEntity rightCycle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(rightStartPose)
                                        .splineToConstantHeading(new Vector2d(-35, 55), Math.toRadians(270))
                                        .forward(27.5)
                                        .splineToSplineHeading(rightPrimaryPose, Math.toRadians(315))
                                        .setReversed(true)
                                        .splineToSplineHeading(rightMiddlePose, Math.toRadians(180))
                                        .splineToLinearHeading(rightStackPose, Math.toRadians(180))
                                        .setReversed(false)
                                        .splineToLinearHeading(rightMiddlePose, Math.toRadians(0))
                                        .splineToSplineHeading(rightPrimaryPose, Math.toRadians(315))
                                        .setReversed(true)
                                        .splineToSplineHeading(rightMiddlePose, Math.toRadians(180))
                                        .splineToLinearHeading(rightStackPose, Math.toRadians(180))
                                        .setReversed(false)
//                                        .splineToLinearHeading(rightMiddlePose, Math.toRadians(0))
//                                        .splineToSplineHeading(rightPrimaryPose, Math.toRadians(315))

                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-40,12,Math.toRadians(270)), Math.toRadians(180))

                                        .strafeRight(16) //1
//                                        .strafeLeft(5) //2
//                                        .strafeLeft(30) //3

                                        .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightCycle)
                .start();
    }
}