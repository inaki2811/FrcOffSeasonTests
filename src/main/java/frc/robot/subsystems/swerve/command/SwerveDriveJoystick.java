// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.command;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveJoystick extends Command {

    private static final double JOYSTICK_DEADZONE = 0.1 * SwerveConstants.MAX_SPEED_MPS;

  private final Swerve swerve;

  
  private final Supplier<Double> xInput;

  private final Supplier<Double> yInput;

  private final Supplier<Double> zInput;


  private final Supplier<Boolean> fieldRelative;

  /** Creates a new SwerveDriveJoystick. */
  public SwerveDriveJoystick(Swerve swerve, Supplier<Double> xInput, Supplier<Double> yInput, Supplier<Double> zInput, Supplier<Boolean> fieldRelative) {
    
    this.swerve = swerve;
    this.xInput = xInput;
    this.yInput = yInput;
    this.zInput = zInput;
    this.fieldRelative = fieldRelative;
    
    addRequirements(swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xInput.get() * SwerveConstants.MAX_SPEED_MPS;
    double ySpeed = yInput.get() * SwerveConstants.MAX_SPEED_MPS;
    double zSpeed = zInput.get() * SwerveConstants.MAX_ANG_SPD;

    applyDeadband(xSpeed, JOYSTICK_DEADZONE);
    applyDeadband(ySpeed, JOYSTICK_DEADZONE);
    applyDeadband(zSpeed, JOYSTICK_DEADZONE);

    ChassisSpeeds chassisSpeeds;

    if (fieldRelative.get()) {

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerve.geRotation2d());
    
    }else{

      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
    
    }

    swerve.drive(chassisSpeeds, false);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  private static double applyDeadband(double value, double deadzone) {
    return Math.abs(value) > deadzone ? value : 0.0;
  }
}
