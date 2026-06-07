// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {


     
    private final SwerveModule frontLeftModule =
      new SwerveModule(SwerveConstants.FL_PWR, SwerveConstants.FL_TUR);

    
    private final SwerveModule frontRightModule =
      new SwerveModule(SwerveConstants.FR_PWR, SwerveConstants.FR_TUR);
    
    private final SwerveModule backLeftModule =
      new SwerveModule(SwerveConstants.BL_PWR, SwerveConstants.BL_TUR);
  
    private final SwerveModule backRightModule =
      new SwerveModule(SwerveConstants.BR_PWR, SwerveConstants.BR_TUR);
    

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(SwerveConstants.WHEELS_BASE_METERS / 2.0, SwerveConstants.WHEELS_WIDTH_METERS / 2.0),
      new Translation2d(SwerveConstants.WHEELS_BASE_METERS / 2.0, -SwerveConstants.WHEELS_WIDTH_METERS / 2.0),
      new Translation2d(-SwerveConstants.WHEELS_BASE_METERS / 2.0, SwerveConstants.WHEELS_WIDTH_METERS / 2.0),
      new Translation2d(-SwerveConstants.WHEELS_BASE_METERS / 2.0, -SwerveConstants.WHEELS_WIDTH_METERS / 2.0)
      );

    private final StructArrayPublisher<SwerveModuleState> swervePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Detected module states", SwerveModuleState.struct)
                    .publish();

    private final StructArrayPublisher<SwerveModuleState> swerveDesiredStatePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("desiredStates", SwerveModuleState.struct)
                    .publish();

    private Pose2d robotPose = new Pose2d();
 
    private SwerveModulePosition[] previouPositions = new SwerveModulePosition[]{
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()


    };
  
  public Swerve() {
    
  }


  @Override
  public void periodic() {


    swervePublisher.set(getSwerveModuleStates());

  } 


  public  SwerveModulePosition[] getSwerveModulePositions(){
      return new SwerveModulePosition[]{
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition(),
      
 
      };
  }

  public SwerveModuleState[] getSwerveModuleStates (){
    return new SwerveModuleState[]{
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState(),
    };


  }

  public double getHeading(){
    return 0;
  }

  public Rotation2d geRotation2d(){
    return robotPose.getRotation();
  }


  public double getAverageWheelSpeed() {
    SwerveModuleState[] states = getSwerveModuleStates();
    double sum = 0.0;
    for (SwerveModuleState state : states) {
        sum += state.speedMetersPerSecond;
    }
    return sum / states.length;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }



  public void stopModules(){
    frontLeftModule.stop();
    frontRightModule.stop();
    backLeftModule.stop();
    backRightModule.stop();
  }

  public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    setStates(moduleStates);
    swerveDesiredStatePublisher.set(moduleStates);
  }

  public void setStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,  SwerveConstants.MAX_SPEED_MPS);
  
    //frontLeftModule.setDesiredState(desiredStates[0]);
    //frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    //backRightModule.setDesiredState(desiredStates[3]);
    
  }

  public void setStatesDirectoSinFiltros(SwerveModuleState[] desiredStates) {
    
    //frontLeftModule.setDesiredState(desiredStates[0]);
    //frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    //backRightModule.setDesiredState(desiredStates[3]);
    
    swerveDesiredStatePublisher.set(desiredStates);
  }

}