// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Velocity;


/** Add your docs here. */
public class SwerveController {

    private final SparkMax driveMotor;

    private final SparkMax turningMotor;

    private final SparkClosedLoopController drivePID;

    private final SparkClosedLoopController turningPID;

    public SwerveController(SwerveIO io){
        this.driveMotor = io.getDriveMotor();
        this.turningMotor = io.getTurningMotor();
        this.drivePID = driveMotor.getClosedLoopController();
        this.turningPID = turningMotor.getClosedLoopController();
        
        configureDriveMotors();
    }


    private void configureDriveMotors() {
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(40);
        driveConfig.voltageCompensation(12.0);
        driveConfig.encoder.positionConversionFactor(SwerveConstants.ROT_2_M);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.ROT_2_M / 60);
        driveConfig.closedLoop.pid(
            SwerveConstants.VEL_KP,
            SwerveConstants.VEL_KI,
            SwerveConstants.VEL_KD
            );
        driveConfig.closedLoop.velocityFF(SwerveConstants.VEL_KV);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    private void configureTurningMotors() {
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig.idleMode(IdleMode.kBrake);
        turningConfig.smartCurrentLimit(40);
        turningConfig.voltageCompensation(12.0);
        turningConfig.absoluteEncoder.positionConversionFactor(2.0 * Math.PI);
        turningConfig.absoluteEncoder.velocityConversionFactor((2.0 * Math.PI) / 60);
        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turningConfig.closedLoop.pid(
            SwerveConstants.POS_KP, 
            SwerveConstants.POS_KI, 
            SwerveConstants.POS_KD
            );
        turningConfig.closedLoop.velocityFF(SwerveConstants.POS_KV);
        turningConfig.closedLoop.positionWrappingEnabled(true);
        turningConfig.closedLoop.positionWrappingInputRange(0.0, 2.0 * Math.PI);
        turningConfig.closedLoop.maxMotion.maxVelocity(SwerveConstants.MAGIC_MOTION_VELOCITY_STR);
        turningConfig.closedLoop.maxMotion.maxAcceleration(SwerveConstants.MAGIC_MOTION_ACCELERATION_STR);


        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void setVelocity (double velocityMps) {
        if (Math.abs(velocityMps) > 0.1) {

            drivePID.setSetpoint(velocityMps, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, 0.0);

        } else {

            driveMotor.stopMotor();

        }
    }

    public void setAngle ( double angleRad) {
        
        double currentAngleRad = turningMotor.getAbsoluteEncoder().getPosition();
        
        if(Math.abs(angleRad - currentAngleRad) > Math.toRadians(1)){

            turningPID.setSetpoint(angleRad, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, angleRad);

        } else {

            driveMotor.stopMotor();

        }
        
    }
}
