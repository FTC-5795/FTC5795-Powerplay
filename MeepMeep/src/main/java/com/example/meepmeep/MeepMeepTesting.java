
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

        RoadRunnerBotEntity BlueLeft1 = new DefaultBotBuilder(meepMeep)
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
                                .lineToSplineHeading(new Pose2d(14,12, Math.toRadians(270)))
                                .back(24)
                                .build()

                );
        RoadRunnerBotEntity BlueLeft2 = new DefaultBotBuilder(meepMeep)
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
                                .lineToSplineHeading(new Pose2d(33,12, Math.toRadians(270)))
                                .back(24)
                                .build()

                );
        RoadRunnerBotEntity BlueLeft3 = new DefaultBotBuilder(meepMeep)
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
                                .lineToSplineHeading(new Pose2d(58,12, Math.toRadians(270)))
                                .back(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueLeft1)
                .addEntity(BlueLeft2)
                .addEntity(BlueLeft3)
                .start();
    }
}
//  .addTemporalMarker(2, () -> {
//                                    Put code here to be run 2 seconds into the trajectory
//                                    Used for actions during with autonomous (like extending slides)