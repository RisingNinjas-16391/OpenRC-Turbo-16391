package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 45, Math.toRadians(90), Math.toRadians(60), 15)
                .setDimensions(15.5, 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, -62, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(29, -5, Math.toRadians(130)), Math.toRadians(110))
                        .build()
                );
                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.80f)
                .addEntity(myBot)
                .start();
    }
}