package frc.subsystems.drivetrain;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Subsystem;
import frc.subsystems.Odometry.CatzRobotTracker;
import frc.subsystems.Odometry.CatzVision;
import frc.robot.CatzConstants;
import frc.robot.Robot;

public class CatzDrivetrain extends Subsystem{
    private static CatzDrivetrain instance = null;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private static CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];
    private static CatzVision limelightVision = CatzVision.getInstance();

    private final SwerveDriveKinematics swerveDriveKinematics;

    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_FRNT_DRIVE_ID = 7;
    private final int RT_BACK_DRIVE_ID = 5;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_FRNT_STEER_ID = 8;
    private final int RT_BACK_STEER_ID = 6;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_FRNT_ENC_PORT = 8;
    private final int RT_BACK_ENC_PORT = 7;

    private double LT_FRNT_OFFSET = 0.0100; 
    private double LT_BACK_OFFSET = 0.0455;
    private double RT_BACK_OFFSET = 0.2572;
    private double RT_FRNT_OFFSET = 0.0284;

    //For turning correction
    private ChassisSpeeds desiredSpeed;
    private Rotation2d targetRotation;
    private final double TURNING_DEADBAND = 2.0; //degrees
    public static final double HEADING_kP = 0.04; //P constant for heading correction

    private CatzDrivetrain()
    {
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        default:
            gyroIO = new GyroIONavX() {}; // TBD does gryo need sim class
        break;
        }

        swerveDriveKinematics = CatzConstants.DriveConstants.swerveDriveKinematics;

        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 2);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[1] = LT_BACK_MODULE;
        swerveModules[2] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;


        resetMagEncs();
        //Reset Mag Enc after startup
        new Thread(() -> {
            try 
            {
                Thread.sleep(1000);
                zeroGyro();
            } 
            catch (Exception e) 
            {}
        }).start();

        targetRotation = new Rotation2d();
    }

    @Override
    public void update()
    {
        ChassisSpeeds correctedSpeeds = correctHeading(desiredSpeed);

        SwerveModuleState[] targetSwerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(correctedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetSwerveModuleStates, CatzConstants.DriveConstants.MAX_SPEED);

        setSwerveModuleStates(targetSwerveModuleStates);

        for(CatzSwerveModule swerveModule : swerveModules)
        {
            swerveModule.update();
        }
    }

    public void setTeleopChassiss(ChassisSpeeds desiredSpeed)
    {
        this.desiredSpeed = desiredSpeed;
    }

    //inputs SwerveModule sates into each module depending on statenumeber
    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CatzConstants.DriveConstants.MAX_SPEED);

        Logger.getInstance().recordOutput("Drive/module states", states);
        Logger.getInstance().recordOutput("Drive/state speed pwr lt frnt", states[0].speedMetersPerSecond);
        Logger.getInstance().recordOutput("Drive/state speed pwr lt back", states[1].speedMetersPerSecond);
        Logger.getInstance().recordOutput("Drive/state speed pwr rt frnt", states[2].speedMetersPerSecond);
        Logger.getInstance().recordOutput("Drive/state speed pwr rt back", states[3].speedMetersPerSecond);
        

        for(int i = 0; i < 4; i++)
        {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    /*************************************************************
     * 
     * Swerve drive state misc methods
     * 
     ***********************************************************/
    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() 
    {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        for(int i = 0; i < 4; i++)
        {
            moduleStates[i] = swerveModules[i].getModuleState();
        }

        return moduleStates;
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }

        return modulePositions;
    }

    public void printModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
            System.out.println(modulePositions[i].distanceMeters);
        }
    }

    /*************************************************************
     * 
     * Misc encoder/gyro methods
     * 
     ***********************************************************/
    private void resetMagEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets()
    {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(CatzSwerveModule module : swerveModules)
        {
            module.initializeOffset();
        }
    }

    public void stopDriving()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.setPower(0.0);
            module.setSteeringPower(0.0);
        }
    }
    
    public double getGyroAngle()
    {
        return gyroInputs.gyroAngle;
    }

    public double getGyroYaw()
    {
        return gyroInputs.gyroYaw;
    }

    //zeros out the gryo to opposite what it's facing due to NavX instalation 180 degrees backwards
    public void zeroGyro()
    {
      gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
    }

    //From 4481
    /**
     * Optimizes the chassis speed that is put into the kinematics object to allow the robot to hold its heading
     * when no angular velocity is input.
     * The robot will therefore correct itself when it turns without telling it to do so.
     *
     * @param desiredSpeed desired chassis speed that is input by the controller
     * @return corrected {@code ChassisSpeeds} which takes into account that the robot needs to have the same heading
     * when no rotational speed is input
     */
    double previousT;
    double offT;
    
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed){

        //Determine time interval
        double currentT = Robot.currentTime.get();
        double dt = currentT - previousT;

        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);

        if (vr > 0.01 || vr < -0.01){
            offT = currentT;
            targetRotation = CatzRobotTracker.getInstance().getCurrentRotation();
            return desiredSpeed;
        }
        if (currentT - offT < 0.5){
            targetRotation = CatzRobotTracker.getInstance().getCurrentRotation();
            return desiredSpeed;
        }

        //Determine target and current heading
        targetRotation =  targetRotation.plus(new Rotation2d(vr * dt));
        Rotation2d currentHeading = CatzRobotTracker.getInstance().getCurrentRotation();

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = targetRotation.minus(currentHeading);

        if (Math.abs(deltaHeading.getDegrees()) < TURNING_DEADBAND){
            return desiredSpeed;
        }

        double correctedVr = deltaHeading.getRadians() / dt * HEADING_kP;

        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }


    /*************************************************************
    * 
    * Misc
    * 
    ***********************************************************/

    @Override
    public void smartDashboard()
    {
        SmartDashboard.putNumber("NavX Gyro Angle", gyroInputs.gyroAngle);
        LT_FRNT_MODULE.smartDashboard();
        LT_BACK_MODULE.smartDashboard();
        RT_FRNT_MODULE.smartDashboard();
        RT_BACK_MODULE.smartDashboard();
    }

    //returns itself for singleton implementation
    public static CatzDrivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDrivetrain();
        }

        return instance;
    }

}
