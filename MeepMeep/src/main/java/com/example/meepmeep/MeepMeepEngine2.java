package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepEngine2 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double poleDepth = 2;

        Pose2d rightStartPose = new Pose2d(-35, 61.5, Math.toRadians(270));
        Pose2d rightPrimaryPose = new Pose2d(-20, 12, Math.toRadians(270));
        Pose2d rightStackPose = new Pose2d(-55,10.5, Math.toRadians(180));

        Pose2d leftStartPose = new Pose2d(35, 61.5, Math.toRadians(270));
        Pose2d leftPrimaryPose = new Pose2d(20, 12, Math.toRadians(270));
        Pose2d leftStackPose = new Pose2d(55,10.5, Math.toRadians(0));

        RoadRunnerBotEntity rightCycle = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(52, 52, Math.toRadians(296), Math.toRadians(296), 11.735)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(rightStartPose)
                                .splineToLinearHeading(rightPrimaryPose, 0)
                                .forward(poleDepth)
                                .back(poleDepth)
//                                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
//                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
//                                .splineToLinearHeading(rightStackPose, Math.toRadians(0))
//                                .back(12)
//                                .lineToSplineHeading(rightPrimaryPose)
//                                .forward(poleDepth)
//                                .back(poleDepth)
                                .lineToSplineHeading(new Pose2d(-30,12, Math.toRadians(180)))
                                .splineToLinearHeading(rightStackPose, Math.toRadians(180))
                                .back(12)
                                .lineToSplineHeading(rightPrimaryPose)
                                .forward(poleDepth)
                                .back(poleDepth)
                                .lineToSplineHeading(new Pose2d(-25,12, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-40,12, Math.toRadians(180)))
                                .splineToLinearHeading(rightStackPose, Math.toRadians(0))
                                .back(30)
                                .lineToSplineHeading(new Pose2d(-10,14.5, Math.toRadians(270)))
                                .build()
                );

        RoadRunnerBotEntity leftCycle = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(52, 52, Math.toRadians(296), Math.toRadians(296), 11.735)
                .setDimensions(13.5, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(leftStartPose)
                                        .splineToLinearHeading(leftPrimaryPose, Math.toRadians(180))
                                        .forward(poleDepth)
                                        .back(poleDepth)
//                                        .lineToSplineHeading(new Pose2d(25,12, Math.toRadians(270)))
//                                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
//                                        .splineToLinearHeading(leftStackPose, Math.toRadians(0))
//                                        .back(12)
//                                        .lineToSplineHeading(leftPrimaryPose)
//                                        .forward(poleDepth)
//                                        .back(poleDepth)
                                        .lineToSplineHeading(new Pose2d(25,12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                        .splineToLinearHeading(leftStackPose, Math.toRadians(0))
                                        .back(12)
                                        .lineToSplineHeading(leftPrimaryPose)
                                        .forward(poleDepth)
                                        .back(poleDepth)
                                        .lineToSplineHeading(new Pose2d(25,12, Math.toRadians(270)))
                                        .lineToSplineHeading(new Pose2d(40,12, Math.toRadians(0)))
                                        .splineToLinearHeading(leftStackPose, Math.toRadians(0))
                                        .back(12)
                                        .lineToSplineHeading(leftPrimaryPose)
                                        .forward(poleDepth)
                                        .back(poleDepth)
                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(rightCycle)
                .addEntity(leftCycle)
                .start();
    }
}
