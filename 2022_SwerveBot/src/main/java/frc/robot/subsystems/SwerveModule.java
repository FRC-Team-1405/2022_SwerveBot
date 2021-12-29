// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
//CTRE deps
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
//WPILIB deps
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */ 

  /** A swerve module must have a drive motor and a steering motor. The drive motor gives the power to 
  the wheel of the swerve module, and the steering motor points the wheel in the direction it should 
  go. It can be thought of as a vector, with the steering motor controlling the direction and the 
  drive motor controlling the magnitude (oh yeah!). */ 

  //Our drive motor
  private final WPI_TalonFX driveMotor; 
  //Our steering motor 
  private final WPI_TalonFX steeringMotor; 
  //Our external encoder for measuring the turns of the steering motor 
  private final CANCoder steeringEncoder;

  private static final double wheelRadius = Units.inchesToMeters(4); 
  private static final double driveMotorEncoderResolution = 2048;  
  //For converting 100 milleseconds (heretofore referred to as 'ms') to seconds 
  private static final double timeConstantForConversion = 10;

  /** A simple conversion formula to turn encoder velocity (sensor units/100ms) to meters per second. 
  To do: add gear reduction to this formula. Our encoder for the drive motor is located above
  the gear reduction for our swerve module. Therefore, the gear reduction of the swerve module must be 
  factored in to our calculation. Currently, it is not. */ 
  private static final double velocityMeters = wheelRadius * 2 * Math.PI * timeConstantForConversion / driveMotorEncoderResolution;
  
  //I feel the constructor is pretty self-explanatory 
  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID) {

    driveMotor = new WPI_TalonFX(driveMotorID); 
    steeringMotor = new WPI_TalonFX(steeringMotorID); 
    steeringEncoder = new CANCoder(steeringEncoderID);
  } 
  /** Returns the current velocity and rotation angle of the swerve module (in meters per second and 
  radians respectively) */
  public SwerveModuleState getState() { 
    return new SwerveModuleState(getVelocityMetersPerSecond(), new Rotation2d(getAngleRadians())); 
  } 
  /** Allows us to command the swervemodule to any given veloctiy and angle, ultimately coming from our
  joystick inputs. */
  public void setDesiredState(SwerveModuleState desiredState) { 
    SwerveModuleState state = 
      //Later, we will create a SwerveModuleState from joystick inputs to use as our desiredState
       SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleRadians())); 
       /*We need this value back in sensor units/100ms to command the falcon drive motor Note: I do not know
       why the two doubles below need to have 'final' access modifiers*/
       final double driveOutput =  state.speedMetersPerSecond / velocityMeters; 
       //We need this value back in sensor units to command the falcon steering motor 
       final double steeringOutput = state.angle.getRadians(); 
       //Now we can command the steering motor and drive motor 
       driveMotor.set(ControlMode.Velocity, driveOutput); 
       steeringMotor.set(ControlMode.MotionMagic, steeringOutput); 
       /** "Motion Magic" is CTRE (the motor controller manufacturer) "mumbo-jumbo" for a profiled 
       position output. We have found from experiment that motors controlled with this control mode 
       tend to experience less mechanical jerk from sudden changes in acceleration, which the mechanical 
       mentors have informed me is harmful to some rather expensive and difficult to replace parts in 
       the swerve modules. */  
  }
  //A getter for the velocity of the drive motor, converted to meters per second.
  public double getVelocityMetersPerSecond(){ 
    return driveMotor.getSelectedSensorVelocity() * velocityMeters;
  } 
  /** A getter for the angle of steering motor in radians. We intend to configure the steering encoder to 
  return radians automatically, so no conversion is needed here. */
  public double getAngleRadians(){
    return steeringEncoder.getPosition();
  } 
}
