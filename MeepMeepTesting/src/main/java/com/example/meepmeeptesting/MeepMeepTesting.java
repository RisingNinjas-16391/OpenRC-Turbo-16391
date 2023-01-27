package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
//                        // Right Path One Cone
                        new Pose2d(35, -62, Math.toRadians(90)))
                        // Init->Cone
                        .strafeTo(new Vector2d(35, -30))
                        .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(10)).setTangent(Math.toRadians(60)).setTangent(Math.toRadians(0))
                        // Cone -> High
                        .strafeTo(new Vector2d(43, -12))
                        .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(135)), Math.toRadians(140)).setTangent(Math.toRadians(135))
                        // High -> Cone
                        .splineToSplineHeading(new Pose2d(43, -12, Math.toRadians(0)), Math.toRadians(0))
                            //.setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(60, -12))
                        // Cone -> High
                        .strafeTo(new Vector2d(43, -12))
                        .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(135)), Math.toRadians(140)).setTangent(Math.toRadians(140))
                         // High -> Cone
                        .splineToSplineHeading(new Pose2d(43, -12, Math.toRadians(0)), Math.toRadians(0))
                            //.setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(60, -12))
                        // Cone -> High
                        .strafeTo(new Vector2d(43, -12))
                        .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(135)), Math.toRadians(140)).setTangent(Math.toRadians(300))
                         // High -> Park
                        .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(270))
                        //.splineToSplineHeading()
//                        // Left Path One Cone
//                        new Pose2d(35, -62, Math.toRadians(90)))
//                        .strafeTo(new Vector2d(35, -30))
//                        .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(190)).setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
//                        .strafeTo(new Vector2d(50, -12))
//                        .splineToSplineHeading(new Pose2d(28, -5, Math.toRadians(45)), Math.toRadians(40)).setTangent(Math.toRadians(220))
//                        .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(270))

//                        // Multicone
//                        new Pose2d(-35, -62, Math.toRadians(90)))
//                        .strafeTo(new Vector2d(-35, -30))
//                        .splineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(190)).setTangent(Math.toRadians(300)).setTangent(Math.toRadians(0))
//                        .strafeTo(new Vector2d(-50, -12))
//                        .splineToSplineHeading(new Pose2d(-28, -5, Math.toRadians(45)), Math.toRadians(40)).setTangent(Math.toRadians(220))
//                        .splineToSplineHeading(new Pose2d(-35, -35, Math.toRadians(90)), Math.toRadians(270))



                        .build()
                );

                meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.80f)
                .addEntity(myBot)
                .start();
    }
}