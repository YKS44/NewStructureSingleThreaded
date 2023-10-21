package frc.autonomous.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.autonomous.actions.*;
import frc.autonomous.paths.Trajectories;

public class TestRoutineSwerve extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180)));//Follow a straight trajectory while turning towards 180 degrees 
    }
}