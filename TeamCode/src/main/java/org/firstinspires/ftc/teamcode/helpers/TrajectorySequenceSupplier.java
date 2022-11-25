package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@FunctionalInterface
public interface TrajectorySequenceSupplier {
    TrajectorySequence getAsTrajectorySequence();
}
