package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@FunctionalInterface
public interface TrajectorySequenceSupplier {
    TrajectorySequence getAsTrajectorySequence();
}
