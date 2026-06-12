// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.SwerveDebugUtil;

public class SwerveModule {

    private final SwerveController controller;
    private final SwerveIO io;
    private static final double CONTROL_PERIOD_SEC = 0.02;
    private static final double DEBUG_UPDATE_INTERVAL_SEC = 0.02;
    private double lastDebugTime = 0.0;

    public SwerveModule(int driveSparkID, int turningSparkID) {
        this.io = new SwerveIO(driveSparkID, turningSparkID);
        this.controller = new SwerveController(this.io);
    }



    public SwerveModuleState getState() {
        double driveSpeed = io.getDriveVelocityMetersPerSecond();
        double turningAngleRad = io.getTurningEncoderRadians();

        return new SwerveModuleState(driveSpeed, new Rotation2d(turningAngleRad));
    }

    public SwerveModulePosition getPosition() {
        double driveDistanceMeters = io.getDrivePositionMeters();
        double turningAngleRad = io.getTurningEncoderRadians();

        return new SwerveModulePosition(driveDistanceMeters, new Rotation2d(turningAngleRad));    
    }

    public void stop() {
        io.stop();
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        Rotation2d encoderRotation = Rotation2d.fromRadians(io.getTurningEncoderRadians());

        double desiredFinalVel = desiredState.speedMetersPerSecond;

        double currentVel = Math.abs(io.getDriveVelocityMetersPerSecond());

        double wantedAcc = (desiredFinalVel - currentVel)  / CONTROL_PERIOD_SEC;

        double wantedDirection = desiredState.angle.getRadians();

        double wantedFrontAcc = wantedAcc * Math.cos(wantedDirection);

        double wantedSideAcc = wantedAcc * -Math.sin(wantedDirection);
        
        double limitedFrontAcc = clamp(wantedFrontAcc, -SwerveConstants.MAX_FRONT_ACCEL, SwerveConstants.MAX_FRONT_ACCEL);

        double limitedSideAcc = clamp(wantedSideAcc, -SwerveConstants.MAX_SIDE_ACCEL, SwerveConstants.MAX_SIDE_ACCEL);

        double limitedAccMagnitude = Math.hypot(limitedFrontAcc, limitedSideAcc);

        double limitedAcc = Math.copySign(limitedAccMagnitude, wantedAcc);

        double limitedDirection = Math.atan2(-limitedSideAcc, limitedFrontAcc);

        double nextWantedVel = currentVel + (limitedAcc * CONTROL_PERIOD_SEC);


        SwerveModuleState limitedState = new SwerveModuleState(nextWantedVel, Rotation2d.fromRadians(limitedDirection));

        limitedState.optimize(encoderRotation);

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 ) {
            io.stop();
        } else {

            double cosineScalar = limitedState.angle.minus(encoderRotation).getCos();
            
            controller.setVelocity(limitedState.speedMetersPerSecond * cosineScalar);
            controller.setAngle(limitedState.angle.getRadians());
        }
        

        int moduleId = io.getDriveMotor().getDeviceId();
        lastDebugTime = SwerveDebugUtil.publishModuleDebug(
                moduleId, 
                desiredState.speedMetersPerSecond, 
                io.getDriveVelocityMetersPerSecond(), 
                nextWantedVel, 
                wantedAcc, limitedAcc, 
                wantedDirection, limitedDirection, 
                lastDebugTime, DEBUG_UPDATE_INTERVAL_SEC
        );


    
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }


}