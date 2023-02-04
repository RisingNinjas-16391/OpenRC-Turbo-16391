package org.firstinspires.ftc.teamcode.drive.commands.Autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Left {

    public static Pose2d leftify(Pose2d pose) {
        return new Pose2d(-pose.getX(), pose.getY(), leftify(pose.getHeading()));
    }

    public static Pose2d leftify(Pose2d pose, boolean inactive) {
        return !inactive ? new Pose2d(-pose.getX(), pose.getY(), leftify(pose.getHeading())) : pose;
    }

    public static Vector2d leftify(Vector2d vector) {
        return new Vector2d(-vector.getX(), vector.getY());
    }

    public static Vector2d leftify(Vector2d vector, boolean inactive) {
        return !inactive ? new Vector2d(-vector.getX(), vector.getY()) : vector;
    }

    public static double leftify(double angle) {
        return Math.PI - angle;
    }

    public static double leftify(double angle, boolean inactive) {
        return !inactive ? Math.PI - angle : angle;
    }
}
