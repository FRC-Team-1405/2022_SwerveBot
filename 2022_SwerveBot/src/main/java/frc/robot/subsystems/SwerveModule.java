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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

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
    return new SwerveModuleState(getVelocityMetersPerSecond(), new Rotation2d(getAngleNormalized())); 
  } 
  /** Allows us to command the swervemodule to any given veloctiy and angle, ultimately coming from our
  joystick inputs. */
  public void setDesiredState(SwerveModuleState desiredState) { 
      //Later, we will create a SwerveModuleState from joystick inputs to use as our desiredState
      // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleNormalized())); 
      SwerveModuleState state = desiredState;   
      SmartDashboard.putNumber("desired angle", state.angle.getDegrees());    
       /*We need this value back in sensor units/100ms to command the falcon drive motor Note: I do not know
       why the two doubles below need to have 'final' access modifiers*/
       //final double driveOutput =  state.speedMetersPerSecond / velocityMeters;
       //We are using speedMetersPerSecond as a percent voltage value from -1 to 1 
       double percentVoltage = state.speedMetersPerSecond; 
       //We need this value back in sensor units to command the falcon steering motor 
       final double steeringOutput = state.angle.getDegrees(); 

      //  final double normalized = getAngleNormalized();
      final double absolute = getAngle();
      double delta = AngleDelta( absolute, state.angle.getDegrees() );

      if (delta > 90.0) {
        delta -= 180.0 ;
        percentVoltage *= -1;
      } else if (delta < -90.0){
        delta += 180.0 ;
        percentVoltage *= -1;
      }
      //  final double delta = steeringOutput - normalized;
      //  final double target = absolute + delta;

      final double target = AngleToEncoder( absolute + delta );
      
      if (steeringEncoder.getDeviceID() == 31) { 
        System.out.printf("Steering %6.2f %6.2f %6.2f", getAngle(), getAngleNormalized(), state.angle.getDegrees());
      }

      //Now we can command the steering motor and drive motor 
      SmartDashboard.putNumber("output", target);
      if ( percentVoltage != 0 ){
        steeringMotor.set(ControlMode.Position, target); 
      }
      driveMotor.set(ControlMode.PercentOutput, percentVoltage); 
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

  public double getAngle(){ 
    return steeringEncoder.getPosition();
  } 
  public double getAngleNormalized(){
    return Math.IEEEremainder(steeringEncoder.getPosition(), 180.0);
  } 

  protected final static int ENCODER_COUNT = 4096;  
  public static int AngleToEncoder(double deg){
      return (int)((double)deg / 360.0 * (double)ENCODER_COUNT);
  }
  public static double EncoderToAngle(int tick){
      return tick / (double)ENCODER_COUNT * 360.0;
  }

  public static double AngleDelta(double current, double target){
    if (current < 0) current += 360.0;
    if (target < 0) target += 360.0;
    double deltaPos = current - target;
    double deltaNeg = target - current;
    if (Math.abs(deltaPos) < Math.abs(deltaNeg))
        return Math.IEEEremainder(deltaPos,360);
    else
        return Math.IEEEremainder(deltaNeg,360);
}


}
