// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */ 

  private static final double DEADBAND = 0.4; 

  private SwerveDrive driveBase; 
  private DoubleSupplier getXSpeed; 
  private DoubleSupplier getYSpeed; 
  private DoubleSupplier getRotationSpeed; 

  public SwerveDriveCommand(DoubleSupplier getXSpeed, 
                            DoubleSupplier getYSpeed, 
                            DoubleSupplier getRotationSpeed, 
                            SwerveDrive driveBase) {
    // Use addRequirements() here to declare subsystem dependencies. 
    this.getXSpeed = getXSpeed; 
    this.getYSpeed = getYSpeed; 
    this.getRotationSpeed = getRotationSpeed; 
    this.driveBase = driveBase; 
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = deadband(getXSpeed.getAsDouble()); 
    double ySpeed = deadband(getYSpeed.getAsDouble()); 
    double rotationSpeed = deadband(getRotationSpeed.getAsDouble()); 

    driveBase.drive(xSpeed, ySpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 
  //We use this function to apply a prespecified deaband to the drive inputs
  private double deadband(double value){ 
    if (Math.abs(value) < DEADBAND) return 0.0; 
    return value; 
  }
}
