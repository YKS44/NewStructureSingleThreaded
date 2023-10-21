package frc.autonomous.actions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.subsystems.Odometry.CatzRobotTracker;
import frc.subsystems.drivetrain.CatzDrivetrain;
import frc.robot.CatzConstants;

// Follows a trajectory
public class TrajectoryFollowingAction implements ActionBase{
    private final double EXTRA_TIME = 0.0;

    private final Timer timer = new Timer();
    private final HolonomicDriveController controller;
    private final CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();

    private final Trajectory trajectory;
    private final Rotation2d targetHeading;

    /**
     * @param trajectory The trajectory to follow
     * @param refHeading The goal heading for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
     */
    public TrajectoryFollowingAction(Trajectory trajectory, Rotation2d targetHeading)
    {
        this.trajectory = trajectory;
        this.targetHeading = targetHeading; // this returns the desired orientation when given the current position (the function itself is given as an argument). But most of the times, it will just give a constant desired orientation.
        // also, why is it called refheading? wouldn't something like targetOrientation be better

        controller = CatzConstants.DriveConstants.holonomicDriveController; // see catzconstants
    }

    // reset and start timer
    @Override
    public void init() {
        timer.reset();
        timer.start();
    }

    // calculates if trajectory is finished
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds() + EXTRA_TIME); //will only work if the code is configured correctly.
    }

    // sets swerve modules to their target states so that the robot will follow the trajectory
    // see catzconstants
    @Override
    public void update() {
        double currentTime = timer.get();
        Trajectory.State goal = trajectory.sample(currentTime);
        
        ChassisSpeeds adjustedSpeed = controller.calculate(robotTracker.getEstimatedPosition(), goal, targetHeading);
        SwerveModuleState[] targetModuleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        driveTrain.setSwerveModuleStates(targetModuleStates);

        Logger.getInstance().recordOutput("Current Position", robotTracker.getEstimatedPosition());
        Logger.getInstance().recordOutput("Target Position", goal.poseMeters);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelW", adjustedSpeed.omegaRadiansPerSecond);
    }

    // stop all robot motion
    @Override
    public void end() {
        timer.stop();

        driveTrain.stopDriving();
        
        System.out.println("trajectory done");
    }
}