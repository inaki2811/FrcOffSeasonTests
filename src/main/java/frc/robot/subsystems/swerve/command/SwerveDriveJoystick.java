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

  private final Swerve swerve;

  
  private final Supplier<Double> xInput;

  private final Supplier<Double> yInput;

  private final Supplier<Double> zInput;


  /** Creates a new SwerveDriveJoystick. */
  public SwerveDriveJoystick(Swerve swerve, Supplier<Double> xInput, Supplier<Double> yInput, Supplier<Double> zInput) {
    
    this.swerve = swerve;
    this.xInput = xInput;
    this.yInput = yInput;
    this.zInput = zInput;
    
    addRequirements(swerve);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = processAxis(xInput, SwerveConstants.MAX_SPEED_MPS);
    double ySpeed = processAxis(yInput, SwerveConstants.MAX_SPEED_MPS);  
    double zSpeed = processAxis(zInput, SwerveConstants.MAX_ANG_SPD);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);

    swerve.drive(chassisSpeeds);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  private static double processAxis (Supplier<Double> axisSupplier, double maxSpeed) {
    double value = axisSupplier.get();

    value = Math.abs(value) > 0.1 ? value : 0;

    return value * maxSpeed;
  }
}
