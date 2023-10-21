package frc.subsystems.Odometry;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.subsystems.Subsystem;
import frc.subsystems.drivetrain.CatzDrivetrain;
import frc.robot.CatzConstants;

public class CatzRobotTracker extends Subsystem{
    // combines SwerveDrivePoseEstimator with april tags to get robot position

    private static CatzRobotTracker instance = null;

    private final CatzDrivetrain driveTrain = CatzDrivetrain.getInstance();
    private final CatzVision visionLimelight =  CatzVision.getInstance();
    
    private SwerveDrivePoseEstimator poseEstimator;

    private CatzRobotTracker()
    {
        driveTrain.resetDriveEncs();
        poseEstimator = new SwerveDrivePoseEstimator(CatzConstants.DriveConstants.swerveDriveKinematics, Rotation2d.fromDegrees(180), driveTrain.getModulePositions(), new Pose2d(0,0,Rotation2d.fromDegrees(0)));
    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions(), pose);
    }
    //METHOD USED IN AUTONOMOUS CONTAINER


    // get position
    public Pose2d getEstimatedPosition()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getCurrentRotation()
    {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    // updates poseEstimator with new measurements
    @Override
    public void update() 
    {
        visionLimelight.coneNodeTracking();

        //apriltag system
        if(visionLimelight.aprilTagInView())
        {
            poseEstimator.addVisionMeasurement(visionLimelight.getLimelightBotPose(), Logger.getInstance().getRealTimestamp()); //TBD need see how to add confidence into the mix
        }

        //pose updates w/ drivetrain
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions());


        Logger.getInstance().recordOutput("Odometry/pose", poseEstimator.getEstimatedPosition());
    }

    @Override
    public void smartDashboard() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void smartDashboard_DEBUG() {
        // TODO Auto-generated method stub
        
    }

    // returns itself
    public static CatzRobotTracker getInstance()
    {
        if(instance == null) 
        {
            instance = new CatzRobotTracker();
        }
        return instance;
    }
}
