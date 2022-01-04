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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */ 
  //I think we can use these values as our speedlimit, if we make them configureable on Shuffleboard
  public static final double maxVelocity = 3.0; //meters per second
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
    //I am making the maxVelocity configurable so we can ajdust our "speedlimit"
    SmartDashboard.putNumber("Speed Limit", maxVelocity); 
    //It may be useful to reset the gyro like this every boot-up. I believe we did this our old code
    gyro.reset();

    Shuffleboard.getTab("Drive Base").add( this );
  }

  public void drive(double xSpeed, double ySpeed, double rotationSpeed){ 
    //var is cheesy but I don't know any better way to do it 
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldOriented() 
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
                                              ySpeed, 
                                              rotationSpeed, 
                                              gyro.getRotation2d()) 
      : new ChassisSpeeds(xSpeed, 
                          ySpeed, 
                          rotationSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxVelocity); 
    
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
