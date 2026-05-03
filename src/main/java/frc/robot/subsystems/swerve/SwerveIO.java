// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class SwerveIO {


    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMax turningMotor;
    private final AbsoluteEncoder turningAbsoluteEncoder;
    private final double absoluteEncoderOffSetRAD;


    public SwerveIO(
            int driveSparkID,
            int turningSparkID,
            double absoluteEncoderOffSetRAD
    ) {
        this.driveMotor = new SparkMax(driveSparkID, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        this.turningMotor = new SparkMax(turningSparkID, MotorType.kBrushless);
        this.turningAbsoluteEncoder = turningMotor.getAbsoluteEncoder();
        this.absoluteEncoderOffSetRAD = absoluteEncoderOffSetRAD;
        resetEncoder();
    }



    public void resetEncoder(){
        driveEncoder.setPosition(0.0);
    }
    


    public double getAbsoluteEncoderRadians(){

        double angleRad = turningAbsoluteEncoder.getPosition() * (2.0 * Math.PI);

        angleRad -= absoluteEncoderOffSetRAD;

        angleRad = Math.IEEEremainder(angleRad, 2.0 * Math.PI);
        if (angleRad < 0) {
            angleRad += 2.0 * Math.PI;
        }

        return angleRad;

    }



    public SparkMax getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getTurningMotor() {
        return turningMotor;
    }

    public double getDriveMotorVelocityPerSecond() {
        return driveEncoder.getVelocity();
    }

    public double getTurningMotorPositionMeters(){
        return turningAbsoluteEncoder.getPosition();
    }
}


