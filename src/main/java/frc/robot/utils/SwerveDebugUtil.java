 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDebugUtil {

    private SwerveDebugUtil(){

    }

    public static double publishModuleDebug(
        int moduleId,
        double desiredFinalVel,
        double currentVel,
        double nextWantedVel,
        double wantedAcc,
        double limitedAcc,
        double wantedDirection,
        double limitedDirection,
        double lastDebugTime,
        double minUpdateInterval
    ){
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastDebugTime < minUpdateInterval) {
            // No ha pasado suficiente tiempo: no publicamos nada
            return lastDebugTime;
        }

        lastDebugTime = currentTime;

        SmartDashboard.putNumber("Wanted Acc" + moduleId, wantedAcc);
        SmartDashboard.putNumber("Limited Acc" + moduleId, limitedAcc);

        SmartDashboard.putNumber("Wanted Side Acc" + moduleId, wantedAcc * Math.cos(wantedDirection));
        SmartDashboard.putNumber("Wanted Front Acc"+ moduleId, wantedAcc * Math.sin(wantedDirection));

        SmartDashboard.putNumber("Limited Side Acc " + moduleId, limitedAcc * Math.cos(limitedDirection));
        SmartDashboard.putNumber("Limited Front Acc " + moduleId, limitedAcc * Math.sin(limitedDirection));

        
        SmartDashboard.putNumber("Wanted Direction " + moduleId, wantedDirection);
        SmartDashboard.putNumber("Limited Direction " + moduleId, limitedDirection);

        
        SmartDashboard.putNumber("Desired Final Vel " + moduleId, desiredFinalVel);
        SmartDashboard.putNumber("Current Vel " + moduleId, currentVel);
        SmartDashboard.putNumber("Next Wanted Vel " + moduleId, nextWantedVel);

        return lastDebugTime;
    }



}
