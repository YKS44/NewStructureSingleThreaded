package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CatzConstants;

public class CatzMathUtils {
    public static double velocityCntsToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double velocityCntsToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = velocityCntsToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
      }
    
      /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
          double lowerBound;
          double upperBound;
          double lowerOffset = scopeReference % 360;
          if (lowerOffset >= 0) {
              lowerBound = scopeReference - lowerOffset;
              upperBound = scopeReference + (360 - lowerOffset);
          } else {
              upperBound = scopeReference - lowerOffset;
              lowerBound = scopeReference - (360 + lowerOffset);
          }
          while (newAngle < lowerBound) {
              newAngle += 360;
          }
          while (newAngle > upperBound) {
              newAngle -= 360;
          }
          if (newAngle - scopeReference > 180) {
              newAngle -= 360;
          } else if (newAngle - scopeReference < -180) {
              newAngle += 360;
          }
          return newAngle;
      }

      public static double getSwerveRotation(double rotAxis)
      {
        if(rotAxis < CatzConstants.DriveConstants.SWERVE_ROTATION_AXIS_DEADBAND)
        {
            return 0.0;
        }
        else
        {
            return CatzConstants.DriveConstants.MAX_ANGULAR_VELOCITY * (rotAxis - (Math.signum(rotAxis) * CatzConstants.DriveConstants.SWERVE_ROTATION_AXIS_DEADBAND)) / (1 - CatzConstants.DriveConstants.SWERVE_ROTATION_AXIS_DEADBAND);
        }
      }
}
