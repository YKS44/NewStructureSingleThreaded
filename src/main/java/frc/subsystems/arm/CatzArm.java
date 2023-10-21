package frc.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.subsystems.Subsystem;
import frc.subsystems.elevator.CatzElevator;
import frc.subsystems.intake.CatzIntake;
import frc.robot.*;
import frc.robot.Robot.mechMode;

public class CatzArm extends Subsystem
{
    private static CatzArm instance = null;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged  inputs = new ArmIOInputsAutoLogged();


    private boolean extendSwitchState = false;

    private boolean highExtendProcess = false;

    private double targetPosition = -999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    private double elevatorPosition = -999.0;

    private boolean armInPosition = false;
    private int numConsectSamples = 0;

    CatzLog data;

    public CatzArm()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new ArmIOReal();
                break;
            case SIM :
                io = null;// new ArmIOSim();
                break;
            default:
                io = new ArmIOReal() {};
                break;
        }

    }

    @Override
    public void update()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("arm", inputs);
    }

    /*-----------------------------------------------------------------------------------------
    *  
    *  cmdProcArm() called by telop periodic in the "main" loop
    *
    *----------------------------------------------------------------------------------------*/
    public void cmdProcArm(boolean armExtend, boolean armRetract,
                            int cmdUpdateState)
    {
        checkLimitSwitches();

        switch(cmdUpdateState)
        {
            case Robot.COMMAND_UPDATE_PICKUP_GROUND_CONE :    
            case Robot.COMMAND_UPDATE_PICKUP_GROUND_CUBE : 
            case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CUBE :
            case Robot.COMMAND_UPDATE_SCORE_LOW_CONE:
            case Robot.COMMAND_UPDATE_SCORE_LOW_CUBE:
                highExtendProcess = false;
                Robot.armControlMode = mechMode.AutoMode;
                io.armSetPickupPosIO();
                armInPosition = false;
                targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_PICKUP;

                
            break;

            case Robot.COMMAND_UPDATE_SCORE_HIGH_CONE:
            case Robot.COMMAND_UPDATE_SCORE_HIGH_CUBE:
                highExtendProcess = true;
                Robot.armControlMode = mechMode.AutoMode;
                armInPosition = false;
                targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_EXTEND;
            break;

            case Robot.COMMAND_UPDATE_STOW           :
            case Robot.COMMAND_UPDATE_PICKUP_SINGLE_CONE :
            case Robot.COMMAND_UPDATE_SCORE_MID_CUBE :
            case Robot.COMMAND_UPDATE_SCORE_MID_CONE :
                highExtendProcess = false;
                Robot.armControlMode = mechMode.AutoMode;
                io.armSetRetractPosIO();
                armInPosition = false;
                targetPosition = CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT;
            break;
        }


        //Manual Control of arm
        if(armExtend == true)
        {
            Robot.armControlMode = mechMode.ManualMode;
            setArmPwr(CatzConstants.ArmConstants.EXTEND_PWR);

          
            highExtendProcess = false;
            
        }
        else if(armRetract == true)
        {
            Robot.armControlMode = mechMode.ManualMode;

            setArmPwr(CatzConstants.ArmConstants.RETRACT_PWR);
          
            highExtendProcess = false;

            
        }
        else if(inputs.isArmControlModePercentOutput)
        {
            setArmPwr(CatzConstants.ArmConstants.MANUAL_CONTROL_PWR_OFF);
        }



        //checks if elevator has cleared mid node before extending arm.
        if(highExtendProcess == true)
        {
            elevatorPosition = CatzElevator.getIntstance().getElevatorEncoder();

            if(DriverStation.isAutonomousEnabled() && Robot.selectedGamePiece == Robot.GP_CONE)//TBD explain why we need to wait for intake in autonomous and when we have a cone
            {
                if(elevatorPosition >= CatzConstants.ArmConstants.POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR && 
                   CatzIntake.getInstance().isIntakeInPos())
                {
                    io.armSetFullExtendPosIO();
                    highExtendProcess = false;
                }
            }
            else
            {
                if(elevatorPosition >= CatzConstants.ArmConstants.POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR)
                {
                    io.armSetFullExtendPosIO();
                    highExtendProcess = false; 
                }
            }
        }


        
        //tracks the current position of the arm
        currentPosition = inputs.armMotorEncoder;
        positionError = currentPosition - targetPosition;
        if  ((Math.abs(positionError) <= CatzConstants.ArmConstants.ARM_POS_ERROR_THRESHOLD) && targetPosition != CatzConstants.ArmConstants.NO_TARGET_POSITION)
        {
            targetPosition = CatzConstants.ArmConstants.NO_TARGET_POSITION;
            numConsectSamples++;
                if(numConsectSamples >= 10)
                {   
                    armInPosition = true;
                }
        }
        else
        {
            numConsectSamples = 0;
        }


        //Logging data
        Logger.getInstance().recordOutput("Arm/targetPosition", targetPosition);
        Logger.getInstance().recordOutput("Arm/positionError", positionError);
        

        // if((DataCollection.chosenDataID.getSelected() == DataCollection.LOG_ID_INTAKE)) 
        // {        
        //     data = new CatzLog(Robot.currentTime.get(), targetPosition, currentPosition, 
        //                                                 positionError,
        //                                                 -999.0, 
        //                                                 //armMtr.getMotorOutputPercent(),
        //                                                             -999.0, -999.0, -999.0, -999.0, -999.0,
        //                                                             -999.0, -999.0, -999.0, -999.0, -999.0,
        //                                                             DataCollection.boolData);
                                    
        //     Robot.dataCollection.logData.add(data);
        // }

        
    }   //End of cmdProcArm()


    /*-----------------------------------------------------------------------------------------
    *  
    *  xxx()
    *
    *----------------------------------------------------------------------------------------*/
    public void checkLimitSwitches()
    {
        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT);
            extendSwitchState = true;
        }
        else
        {
            extendSwitchState = false;
        }


    }

    public void setArmPwr(double pwr)
    {        
        io.setArmPwrIO(pwr);
    }

    public double getArmEncoder()
    {
        return inputs.armMotorEncoder;
    }

    @Override
    public void smartDashboard()
    {
        SmartDashboard.putNumber("arm encoder position", inputs.armMotorEncoder);
    }

    public boolean isArmInPos()
    {
        return armInPosition;
    }

    //returns itself for singleton implementation
    public static CatzArm getInstance()
    {
        if(instance == null)
        {
            instance = new CatzArm();
        }

        return instance;
    }
}
