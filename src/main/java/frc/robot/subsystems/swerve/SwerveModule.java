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
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            io.stop();
            return;
        }

        Rotation2d encoderRotation = Rotation2d.fromRadians(io.getTurningEncoderRadians());


        
    
        //desiredState.optimize(encoderRotation);
        controller.setVelocity(desiredState.speedMetersPerSecond);
        controller.setAngle(desiredState.angle.getRadians());

        int moduleId = io.getDriveMotor().getDeviceId();
        lastDebugTime = SwerveDebugUtil.publishModuleDebug(
                moduleId, desiredState.speedMetersPerSecond, io.getDriveVelocityMetersPerSecond(), desiredState.speedMetersPerSecond,
                0, 0, desiredState.angle.getRadians(), desiredState.angle.getRadians(),
                lastDebugTime, DEBUG_UPDATE_INTERVAL_SEC
        );


    }




}