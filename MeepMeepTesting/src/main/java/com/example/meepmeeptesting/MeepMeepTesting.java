package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static Pose2d leftify(Pose2d pose) {
        return new Pose2d(-pose.getX(), pose.getY(), leftify(pose.getHeading()));
    }
    public static Vector2d leftify(Vector2d vector) {
        return new Vector2d(-vector.getX(), vector.getY());
    }
    public static double leftify (double angle) {
        return Math.PI - angle;
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 45, Math.toRadians(90), Math.toRadians(60), 15)
                .setDimensions(15.5, 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
//                        // Right Path One Cone
                        leftify(new Pose2d(35, -62, Math.toRadians(90))))
                        .strafeTo(leftify(new Vector2d(35, -24)))
                        .splineToSplineHeading(leftify(new Pose2d(60.7, -11, Math.toRadians(0))), leftify(Math.toRadians(5)))
                        .setTangent(leftify(Math.toRadians(60))).setTangent(leftify(Math.toRadians(0)))
                        .strafeTo(leftify(new Vector2d(42, -12)))
                        .splineToSplineHeading(leftify(new Pose2d(26, -3, Math.toRadians(135))), leftify(Math.toRadians(140))).setTangent(Math.toRadians(315))

                        .build()
                );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.80f)
                .addEntity(myBot)
                .start();
    }
}