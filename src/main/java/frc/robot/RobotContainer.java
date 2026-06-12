// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.command.SwerveDriveJoystick;

public class RobotContainer {

    private final Swerve swerve;

    private final XboxController driverController;

    
    public RobotContainer(){
        this.swerve = new Swerve();
        this.driverController = new XboxController(0);

        swerve.setDefaultCommand(new SwerveDriveJoystick(
            swerve,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()
            )
        );
            
    }


    
}
