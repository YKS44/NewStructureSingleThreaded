package frc.autonomous.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.autonomous.actions.*;
import frc.autonomous.paths.Trajectories;

public class TestRoutineConnect extends AutonRoutineBase{
    @Override
    protected void routine() {
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryConnect1,Rotation2d.fromDegrees(0))); //Follow a trajectory, and follow another trajectory. 
        runAction(new TrajectoryFollowingAction(Trajectories.testTrajectoryConnect2,Rotation2d.fromDegrees(0)));
    }
}