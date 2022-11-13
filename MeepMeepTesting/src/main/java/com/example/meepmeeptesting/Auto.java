package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class Auto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(92), Math.toRadians(92), 14.173)
                .setDimensions(17.765, 17.008)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35, -34,  Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(-35, -12,  Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-32, -7,  Math.toRadians(45)))
                                .waitSeconds(0.5)

                                .lineToLinearHeading(new Pose2d(-35, -12,  Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-58, -12,  Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(-35, -12,  Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-32, -7,  Math.toRadians(45)))
                                .waitSeconds(0.5)

                                .lineToLinearHeading(new Pose2d(-35, -12,  Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-58, -12,  Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(-35, -12,  Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-32, -7,  Math.toRadians(45)))
                                .waitSeconds(0.5)
                                // Left Park
//                                .lineToLinearHeading(new Pose2d(-10, -34,  Math.toRadians(90)))
                                // Center Park
//                                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(-34, -34,  Math.toRadians(90)))
                                // Right Park
                                .lineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-58, -20,  Math.toRadians(90)))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.80f)
                .addEntity(myBot)
                .start();
    }
}
