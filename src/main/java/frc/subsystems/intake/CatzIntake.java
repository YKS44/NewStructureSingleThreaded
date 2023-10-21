package frc.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Robot.mechMode;
import frc.subsystems.Subsystem;
import frc.DataLogger.*;

public class CatzIntake extends Subsystem
{
    private static CatzIntake instance = null;
    
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private PIDController pid;
    
    private Boolean  pidEnable = false;

    private double   targetPositionDeg = CatzConstants.IntakeConstants.STOW_ENC_POS;

    private double   targetPower = 0.0;
    private double   prevTargetPwr = 0.0;

    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    private double prevCurrentPosition = -999.0;

    private boolean intakeInPosition = false;

    private double pidPower = 0.0;
    private double ffPower = 0.0;

    private int numConsectSamples = 0;



    //private double wristAngle;


    /*----------------------------------------------------------------------------------------------
    *
    *  Misc
    *
    *---------------------------------------------------------------------------------------------*/
    CatzLog data;

    /*----------------------------------------------------------------------------------------------
    *
    *  CatzIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    private CatzIntake()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new IntakeIOReal();
                break;
            case SIM :
                io = null; //new IntakeIOSim();
                break;
            default:
                io = new IntakeIOReal() {};
                break;
        }
    
        pid = new PIDController(CatzConstants.IntakeConstants.GROSS_kP, CatzConstants.IntakeConstants.GROSS_kI, CatzConstants.IntakeConstants.GROSS_kD);
       
    }

    @Override
    public void update()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("intake", inputs);
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  cmdProcIntake()
    *
    *---------------------------------------------------------------------------------------------*/
    public void cmdProcIntake(double wristPwr, boolean rollersIn, boolean rollersOut, boolean manualMode, 
                                                                                      boolean softLimitOverride, 
                                                                                      int CmdStateUpdate, 
                                                                                      int gamePiece)
    {
        //Manual Control of Intake
        if(manualMode){                
            pidEnable = false;
            Robot.intakeControlMode = mechMode.ManualMode;
        }

        if(Math.abs(wristPwr) >= 0.1)//if we are apply wrist power manually
        {
            if (pidEnable == true)//check if in manual holding state
            {
                Robot.intakeControlMode = mechMode.ManualHoldMode;

                if(wristPwr > 0)
                {
                    targetPositionDeg = Math.min((targetPositionDeg + wristPwr * CatzConstants.IntakeConstants.MANUAL_HOLD_STEP_SIZE), CatzConstants.IntakeConstants.SOFT_LIMIT_FORWARD);
                }
                else
                {
                    targetPositionDeg = Math.max((targetPositionDeg + wristPwr * CatzConstants.IntakeConstants.MANUAL_HOLD_STEP_SIZE), CatzConstants.IntakeConstants.SOFT_LIMIT_REVERSE);
                }
                prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
            }
            else //in full manual mode
            {
                targetPower = wristPwr * CatzConstants.IntakeConstants.WRIST_MAX_PWR;    
                io.wristSetPercentOuputIO(targetPower);
            }
        }
        else //Manual power is OFF
        {
            if(pidEnable == false)//if we are still in manual mode and want to hold intake in place
            {
                targetPower = 0.0;
                io.wristSetPercentOuputIO(targetPower);
            }
        }

        

        //Set state control of Intake
        if(CmdStateUpdate != Robot.COMMAND_STATE_NULL)
        {
            pid.reset();
            pidEnable = true;
            intakeInPosition = false;
            Robot.intakeControlMode = mechMode.AutoMode;
            prevCurrentPosition = -prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values

            switch(CmdStateUpdate)
            {
                case Robot.COMMAND_UPDATE_STOW :
                    targetPositionDeg = CatzConstants.IntakeConstants.STOW_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE :
                    targetPositionDeg = CatzConstants.IntakeConstants.INTAKE_CONE_ENC_POS_GROUND;
                    break;
                    
                case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE :
                    targetPositionDeg = CatzConstants.IntakeConstants.INTAKE_CUBE_ENC_POS;
                    break;

                case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE :
                    targetPositionDeg = CatzConstants.IntakeConstants.INTAKE_CONE_ENC_POS_SINGLE;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CONE :
                    targetPositionDeg = CatzConstants.IntakeConstants.SCORE_CONE_LOW_ENC_POS;
                    Timer.delay(0.1);//wait for arm to deploy -TBD is this a temp fix
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_MID_CUBE :
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                    targetPositionDeg = CatzConstants.IntakeConstants.SCORE_CUBE_ENC_POS;
                    break;
    
                case Robot.COMMAND_UPDATE_SCORE_MID_CONE :
                    targetPositionDeg = CatzConstants.IntakeConstants.SCORE_CONE_MID_ENC_POS;
                    break;
                    
                case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE :
                    targetPositionDeg = CatzConstants.IntakeConstants.SCORE_CONE_HIGH_ENC_POS;
                    break;

                default:
                    pidEnable = false;
                    //TBD
                    break;
            }
        }

        //Pid controller that determines wheather the intake is in position and calculated a target pwr with pid and gravity
        if(pidEnable)   
        {
            //----------------------------------------------------------------------------------
            //  Chk if at final position
            //----------------------------------------------------------------------------------
            currentPosition = inputs.wristPosEnc / CatzConstants.IntakeConstants.WRIST_CNTS_PER_DEGREE;

            positionError = currentPosition - targetPositionDeg;


            if  ((Math.abs(positionError) <= CatzConstants.IntakeConstants.INTAKE_POS_ERROR_THRESHOLD_DEG))
            {
                numConsectSamples++;
                if(numConsectSamples >= 1)  //-TBD how can we raise the counter?
                {   
                    intakeInPosition = true;
                }
            }
            else
            {
                numConsectSamples = 0;
            }
            
            
            if(Math.abs(positionError) >= CatzConstants.IntakeConstants.PID_FINE_GROSS_THRESHOLD_DEG)
            {
                pid.setP(CatzConstants.IntakeConstants.GROSS_kP);
                pid.setI(CatzConstants.IntakeConstants.GROSS_kI);
                pid.setD(CatzConstants.IntakeConstants.GROSS_kD);
            }
            else if(Math.abs(positionError) < CatzConstants.IntakeConstants.PID_FINE_GROSS_THRESHOLD_DEG)
            {
                pid.setP(CatzConstants.IntakeConstants.FINE_kP);
                pid.setI(CatzConstants.IntakeConstants.FINE_kI);
                pid.setD(CatzConstants.IntakeConstants.FINE_kD);
            }

            pidPower = pid.calculate(currentPosition, targetPositionDeg);
            ffPower = calculateGravityFF();
            targetPower = pidPower + ffPower;

            //-------------------------------------------------------------
            //  checking if we did not get updated position value(Sampling Issue).
            //  If no change in position, this give invalid target power(kD issue). -TBD shouldn't d term zero out?
            //  Therefore, go with prev targetPower Value.
            //-------------------------------------------------------------------
            if(prevCurrentPosition == currentPosition)
            {
                targetPower = prevTargetPwr;
            }

            //----------------------------------------------------------------------------------
            //  If we are going to Stow Position & have passed the power cutoff angle, set
            //  power to 0, otherwise calculate new motor power based on position error and 
            //  current angle
            //----------------------------------------------------------------------------------
            if(targetPositionDeg == CatzConstants.IntakeConstants.STOW_ENC_POS && currentPosition > CatzConstants.IntakeConstants.STOW_CUTOFF)
            {
                targetPower = 0.0;
            }

            io.wristSetPercentOuputIO(targetPower);

            prevCurrentPosition = currentPosition;
            prevTargetPwr = targetPower;
           
        }

        //Roller Commands depending on if cube or cone
        if(rollersIn)
        {
            if(gamePiece == Robot.GP_CUBE)
            {
                rollersInCube();
            } 
            else 
            {
                rollersInCone();
            }
        }
        else if(rollersOut)
        {
            if(gamePiece == Robot.GP_CUBE)
            {
                rollersOutCube();
            } 
            else 
            {
                rollersOutCone();
            }
        }
        else
        {
            rollersOff();
        }

        

        //overides softlimits by drivers
        if(softLimitOverride){
            io.intakeConfigureSoftLimitOverride(false);
        }
        else{
            io.intakeConfigureSoftLimitOverride(true);
        }

        //Logger outputs 
        Logger.getInstance().recordOutput("Intake/targetPosDeg", targetPositionDeg);
        Logger.getInstance().recordOutput("Intake/current Position", currentPosition);
        Logger.getInstance().recordOutput("Intake/targetPower", targetPower);
        Logger.getInstance().recordOutput("Intake/ffPower", ffPower);
        Logger.getInstance().recordOutput("Intake/pidPower", pidPower);

        // if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)) 
        // {        
        //     data = new CatzLog(Robot.currentTime.get(), targetPositionDeg, currentPosition, 
        //                                                 targetPower, 
        //                                                 pidPower,
        //                                                 ffPower,
        //                                                 -999.0,
        //                                                 //wristMtr.getMotorOutputPercent(),
        //                                                             -999.0, -999.0, -999.0, 
        //                                                             -999.0, -999.0, -999.0, -999.0, -999.0,
        //                                                             DataCollection.boolData);
                                    
        //     Robot.dataCollection.logData.add(data);
        // }
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - PID 
    *
    *---------------------------------------------------------------------------------------------*/
    public void resetPID(){
        pidEnable = false; //-TBD do we still need this
        pid.reset();
    }

    public void enablePID(boolean set){
        pidEnable = set;
    }

    public boolean getPIDEnabled(){
        return pidEnable;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Rollers
    *
    *---------------------------------------------------------------------------------------------*/
    public void rollersOff()
    {
        io.rollersOffIO();
    }

    public void rollersInCube()
    {
        io.rollersOnIO(CatzConstants.IntakeConstants.ROLLERS_PWR_CUBE_IN);
    }

    public void rollersOutCube()
    {
        io.rollersOnIO(CatzConstants.IntakeConstants.ROLLERS_PWR_CUBE_OUT);
    }

    public void rollersInCone()
    {
        io.rollersOnIO(CatzConstants.IntakeConstants.ROLLERS_PWR_CONE_IN);
    }

    public void rollersOutCone()
    {
        io.rollersOnIO(CatzConstants.IntakeConstants.ROLLERS_PWR_CONE_OUT);
    }
    
    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - Wrist
    *
    *---------------------------------------------------------------------------------------------*/
    public double calcWristAngle()
    {
        double wristAngle = ((inputs.wristPosEnc / CatzConstants.IntakeConstants.WRIST_CNTS_PER_DEGREE) - CatzConstants.IntakeConstants.WRIST_ABS_ENC_OFFSET_DEG);
        return wristAngle;
    }

    public double getWristPosition(){
        return inputs.wristPosEnc;
    }
    
    public double calculateGravityFF()
    {
        double radians = Math.toRadians(calcWristAngle() - CatzConstants.IntakeConstants.CENTER_OF_MASS_OFFSET_DEG);
        double cosineScalar = Math.cos(radians);
        
        return CatzConstants.IntakeConstants.MAX_GRAVITY_FF * cosineScalar;
    }

    public double intakeWristTemp()
    {
        return inputs.wristTemp;
    }
    
    @Override
    public void smartDashboard_DEBUG()
    {
        SmartDashboard.putNumber ("wrist ang",  calcWristAngle());
        SmartDashboard.putNumber ("GravityFF",       calculateGravityFF());
        SmartDashboard.putNumber ("IntakeClosedLoopError", pid.getPositionError());
        SmartDashboard.putNumber ("applied output",  inputs.wristAppliedPwr);
        SmartDashboard.putBoolean("pid",             pidEnable);
        SmartDashboard.putNumber ("mtr abs", inputs.wristPosEnc);
    }

    public boolean isIntakeInPos()
    {
        return intakeInPosition;
    }

    //returns itself for singleton implementation
    public static CatzIntake getInstance()
    {
        if(instance == null)
        {
            instance = new CatzIntake();
        }

        return instance;
    }


}