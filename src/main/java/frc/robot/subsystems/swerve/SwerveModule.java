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
    private static final double DEBUG_UPDATE_INTERVAL_SEC = 1.0;
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

    public void setDesiredState(SwerveModuleState desiredState, boolean isPathPlannerAttached) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            io.stop();
            return;
        }

        Rotation2d encoderRotation = Rotation2d.fromRadians(io.getTurningEncoderRadians());

        if (!isPathPlannerAttached) {
            double desiredFinalVel = desiredState.speedMetersPerSecond;
            double currentVel = Math.abs(io.getDriveVelocityMetersPerSecond());
            double wantedDirection = desiredState.angle.getRadians();
            
            double wantedAcc = (desiredFinalVel - currentVel) / CONTROL_PERIOD_SEC;

            double[] accLimits = accLimits(wantedAcc, wantedDirection, desiredFinalVel);
            double limitedAcc = accLimits[0];
            double limitedDirection = accLimits[1];

            double nextWantedVel = currentVel + (limitedAcc * CONTROL_PERIOD_SEC);

            SwerveModuleState optimizeState = new SwerveModuleState(nextWantedVel, Rotation2d.fromRadians(limitedDirection));
            optimizeState.optimize(encoderRotation);

            controller.setVelocity(optimizeState.speedMetersPerSecond);
            controller.setAngle(optimizeState.angle.getRadians());

            int moduleId = io.getDriveMotor().getDeviceId();
            lastDebugTime = SwerveDebugUtil.publishModuleDebug(
                    moduleId, desiredFinalVel, currentVel, nextWantedVel,
                    wantedAcc, limitedAcc, wantedDirection, limitedDirection,
                    lastDebugTime, DEBUG_UPDATE_INTERVAL_SEC
            );

        } else {
            desiredState.optimize(encoderRotation);
            controller.setVelocity(desiredState.speedMetersPerSecond);
            controller.setAngle(desiredState.angle.getRadians());
        }
    }

    private double[] accLimits(double wantedAcc, double wantedDirection, double desiredFinalVel) {
        double wantedAccMagnitude = Math.abs(wantedAcc);

        double maxForwardAccel = SwerveConstants.MAX_FORDWARD_ACCEL * (1.0 - (io.getDriveVelocityMetersPerSecond() / desiredFinalVel));
        double forwardAccel = Math.min(wantedAccMagnitude, maxForwardAccel);
        
        double skidAccel = Math.min(wantedAccMagnitude, SwerveConstants.MAX_SKID_ACCEL);
        double minAccel = Math.min(skidAccel, forwardAccel);
        
        double wantedSideAcc = minAccel * -Math.sin(wantedDirection);
        double wantedFrontAcc = minAccel * Math.cos(wantedDirection);

        double limitedFrontAcc = clamp(wantedFrontAcc, -SwerveConstants.MAX_FRONT_ACCEL, SwerveConstants.MAX_FRONT_ACCEL);
        double limitedSideAcc = clamp(wantedSideAcc, -SwerveConstants.MAX_SIDE_ACCEL, SwerveConstants.MAX_SIDE_ACCEL);

        double limitedAcc = Math.hypot(limitedFrontAcc, limitedSideAcc);
        final double limitedDirection;
        
        if (Math.abs(limitedSideAcc) < SwerveConstants.MAX_SIDE_ACCEL && Math.abs(limitedFrontAcc) < SwerveConstants.MAX_FRONT_ACCEL) {
            limitedDirection = wantedDirection;
        } else {
            limitedDirection = Math.atan2(-limitedSideAcc, limitedFrontAcc);
        }

        return new double[] { Math.copySign(limitedAcc, wantedAcc), limitedDirection };
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}