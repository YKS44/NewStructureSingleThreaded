package frc.autonomous.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.autonomous.actions.*;
import frc.autonomous.paths.Trajectories;

public class TestRoutineCurve extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryCurve, Rotation2d.fromDegrees(0))); //Follow a curved trajectory
    }
}