// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */ 
  public static final double maxSpeed = 3.0; 
  public static final double maxAngularSpeed = Math.PI;
  
  /** These variables store the location of each swerve module relative to the center of the robot. 
  Currently, I am just copying the ones from the example code. */
  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); 
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381); 
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381); 
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381); 
  //Our swerve modules 
  private final SwerveModule frontLeft = new SwerveModule(1, 5, 9); 
  private final SwerveModule frontRight = new SwerveModule(2, 6, 10); 
  private final SwerveModule backLeft = new SwerveModule(3, 7, 11); 
  private final SwerveModule backRight = new SwerveModule(4, 8, 12); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); 
  
  private final SwerveDriveKinematics kinematics = 
          new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, 
          backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = 
          new SwerveDriveOdometry(kinematics, gyro.getRotation2d()); 

  public SwerveDrive() {
    gyro.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed, 
  double speedLimit){ 
    //var is cheesy but I don't know any better way to do it 
    var swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldOriented() 
      //unsure of whether the speedlimits should be multiplied here or somewhere else
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * speedLimit, 
                                              ySpeed * speedLimit, 
                                              rotationSpeed * speedLimit, 
                                              gyro.getRotation2d()) 
      : new ChassisSpeeds(xSpeed * speedLimit, 
                          ySpeed * speedLimit, 
                          rotationSpeed * speedLimit)); 
    
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxSpeed); 
    
    frontLeft.setDesiredState(swerveModuleStates[0]); 
    frontRight.setDesiredState(swerveModuleStates[1]); 
    backLeft.setDesiredState(swerveModuleStates[2]); 
    backRight.setDesiredState(swerveModuleStates[3]); 
    
  } 

public void updateOdometry(){ 
  odometry.update(gyro.getRotation2d(), frontLeft.getState(), 
                                        frontRight.getState(), 
                                        backLeft.getState(), 
                                        backRight.getState());
  } 

  
  public boolean fieldOriented(){ 
    return gyro != null ? true : false;
  }

}
