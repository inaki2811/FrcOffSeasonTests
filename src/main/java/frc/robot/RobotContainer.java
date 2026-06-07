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
            
        new JoystickButton(driverController, XboxController.Button.kA.value)
            .whileTrue(
                new RunCommand(() -> {

                    double time = Timer.getFPGATimestamp();
                    double amplitud = 3.8;
                    double frecuencia = 3.0; 
                    double velocidadSenoidal = amplitud * Math.sin(time * frecuencia);

                    SwerveModuleState estadoForzado = new SwerveModuleState(velocidadSenoidal, new Rotation2d(0));
                    swerve.setStatesDirectoSinFiltros(new SwerveModuleState[] {
                        estadoForzado, estadoForzado, estadoForzado, estadoForzado
                    });
                },
                swerve)
            );

        new JoystickButton(driverController, XboxController.Button.kB.value)
            .whileTrue(
                new RunCommand(() -> {

                    double amplitud = 3.8; 

                    double frecuencia = 0.5; 
            
                    double tiempoModulo = (Timer.getFPGATimestamp() * frecuencia) % 2.0;
                    double velocidadCuadrada = 0.0;

                    if (tiempoModulo < 1.0) {
                        velocidadCuadrada = amplitud; 
                    } else {
                        velocidadCuadrada = -amplitud; 
                    }

                    SwerveModuleState estadoForzado = new SwerveModuleState(velocidadCuadrada, new Rotation2d(0));
                    swerve.setStatesDirectoSinFiltros(new SwerveModuleState[] {
                        estadoForzado, estadoForzado, estadoForzado, estadoForzado
                    });
                },
                swerve)
            );
    }


    
}
