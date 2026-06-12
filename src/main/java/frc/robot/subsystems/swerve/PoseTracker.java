// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class PoseTracker {

private Pose2d pose2d;

private SwerveModulePosition[] previousPositions;

private final SwerveDriveKinematics swerveKinematics;


public PoseTracker(SwerveModulePosition[] previousPositions, SwerveDriveKinematics swerveDriveKinematics){
    this.pose2d = new Pose2d(); 
    this.previousPositions = previousPositions;    
    this.swerveKinematics = swerveDriveKinematics;
}

public void update(SwerveModulePosition[] currentPosition){

    Twist2d twist = swerveKinematics.toTwist2d(previousPositions, currentPosition);

    pose2d = pose2d.exp(twist);
    
    previousPositions = currentPosition;

}

public Pose2d getPose2d(){
    return pose2d;
}

}
