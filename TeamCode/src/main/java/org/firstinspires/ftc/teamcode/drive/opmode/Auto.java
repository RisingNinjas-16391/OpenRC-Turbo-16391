package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Autonomous", group="Autonomous")
//@Disabled

public class Auto extends AutonomousTemplate {
    Hardware robot = new Hardware();   //Uses heavily modified untested hardware

    public void initialize() {
        robot.init(hardwareMap);
    }

    public void regularAutonomous() {
        //TODO: Trajectories for regular
    }

    public void pathA() {
        //TODO: Trajectories for A auto
    }

    public void pathB() {
        //TODO: Trajectories for B auto
    }

    public void pathC() {
        //TODO: Trajectories for C auto
    }


}