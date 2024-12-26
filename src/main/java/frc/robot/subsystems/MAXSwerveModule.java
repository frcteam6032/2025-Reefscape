// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  // Create new instance of the spark maxes 
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;


  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  
   // https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    // The docs specified a migration for the relative encoders but not the absolute encoders

    SparkMaxConfig drivingConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    // Create the configurations for the spark max
    drivingConfig
    .idleMode(ModuleConstants.kDrivingMotorIdleMode)
    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    drivingConfig.encoder
    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    drivingConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD, ModuleConstants.kDrivingFF)
    .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    turningConfig
    .idleMode(ModuleConstants.kTurningMotorIdleMode)
    .inverted(ModuleConstants.kTurningEncoderInverted)
    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    turningConfig.encoder
    .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    drivingConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pidf(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, ModuleConstants.kTurningFF)
    .positionWrappingEnabled(true)
    .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
    .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
    .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);
    
  
    // Burn the configurations to the memory 
    m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition());
    m_drivingSparkMax.getEncoder().setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingSparkMax.getEncoder().getVelocity()
    ,
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  // CHANGE FROM NEGATIVE BACK TO POSITIVE
  
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        -m_drivingSparkMax.getEncoder().getPosition(),
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningSparkMax.getAbsoluteEncoder().getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.

    m_drivingSparkMax.getClosedLoopController().setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningSparkMax.getClosedLoopController().setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingSparkMax.getEncoder().setPosition(0);
  }
}
