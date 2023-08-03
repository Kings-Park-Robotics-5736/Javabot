// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {

  private final WPI_TalonFX m_driveMotor;
  private final int m_id;
  private final WPI_TalonFX m_turningMotor;

  private final WPI_CANCoder m_turningEncoder;
  private final PIDController m_drivePIDController = new PIDController(DriveConstants.kPDrive, 0, 0);
  private final PIDController m_turnPIDController = new PIDController(5, 0, 0);
  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVoltsDrive , DriveConstants.kvVoltsDrive,DriveConstants.kaVoltsDrive);

  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(DriveConstants.ksVoltsTurning,
      DriveConstants.kvVoltSecondsPerMeterTurning,
      DriveConstants.kaVoltSecondsSquaredPerMeterTurning);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderChannels   The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      String canName,
      boolean invertTurning,
      boolean invertDrive,
      double turningEncoderOffset) {


    m_id= driveMotorChannel;
    m_driveMotor = new WPI_TalonFX(driveMotorChannel, canName);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel, canName);

    m_turningMotor.setInverted(invertTurning);
    m_driveMotor.setInverted(invertDrive);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    m_turningEncoder = new WPI_CANCoder(turningEncoderChannel, canName);

    m_turningEncoder.configFactoryDefault();
    m_turningEncoder.configMagnetOffset(turningEncoderOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double degressToRadians(double degrees) {
    return degrees * Math.PI / 180;
  }

  private double getTurnEncoderPositionInRadians() {
    double retVal = degressToRadians(m_turningEncoder.getAbsolutePosition());
    return retVal;
  }

  private double getDriveEncoderVelocity() {
    //WARNING - unit from falcon encoder is per 100ms, so need to multiply by 10 to get to per 1s
    return m_driveMotor.getSelectedSensorVelocity() * 10 * (2 * Math.PI * ModuleConstants.kWheelRadius)
        / (ModuleConstants.kEncoderResolution * ModuleConstants.kDriveGearRatio);
  }

  private double getDriveEncoderPosition() {
    return m_driveMotor.getSelectedSensorPosition() * (2 * Math.PI * ModuleConstants.kWheelRadius)
        / (ModuleConstants.kEncoderResolution * ModuleConstants.kDriveGearRatio);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocity(), new Rotation2d(getTurnEncoderPositionInRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderPosition(), new Rotation2d(getTurnEncoderPositionInRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(getTurnEncoderPositionInRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getDriveEncoderVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turnPIDController.calculate(getTurnEncoderPositionInRadians(),
        state.angle.getRadians());

    SmartDashboard.putNumber("Error " + m_id,getDriveEncoderVelocity() -state.speedMetersPerSecond );
    SmartDashboard.putNumber("velcity set " +m_id,getDriveEncoderVelocity() );
 
    SmartDashboard.putNumber("velcity req " +m_id,state.speedMetersPerSecond );

    SmartDashboard.putNumber("turn set " +m_id,getTurnEncoderPositionInRadians() );
 
    SmartDashboard.putNumber("turn req " +m_id,state.angle.getRadians());


    final double turnFeedforward =0;// m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }


  public void forceStop(){
    m_driveMotor.setVoltage(0);
    m_turningMotor.setVoltage(0);
  }
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
  }
}
