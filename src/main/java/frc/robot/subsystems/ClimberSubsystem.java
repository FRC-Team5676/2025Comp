// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class ClimberSubsystem extends SubsystemBase {
  
  private double m_targetRadians;

  private static final double kGearRatio = 45/1;
  private static final double kIntakeArmEncoderPositionFactor = (2 * Math.PI) / kGearRatio;

  private final int m_ClimberCanId = 56;

  private final RelativeEncoder m_driveEncoder;
  private final SparkMax m_driveMotor;
  private final SparkClosedLoopController m_driveController;

  private final double minRadians = Units.degreesToRadians(-1000);
  private final double maxRadians = Units.degreesToRadians(1000);

  public ClimberSubsystem() {
    // Drive Motor setup
    m_driveMotor = new SparkMax(m_ClimberCanId, MotorType.kBrushless);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .p(0.5)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    config.encoder.positionConversionFactor(kIntakeArmEncoderPositionFactor);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_targetRadians = m_driveEncoder.getPosition();

    ShuffleboardContent.initClimber(this);
  }

  @Override
  public void periodic() {
  }

  public double getMinDegrees() {
    return Units.radiansToDegrees(minRadians);
  }

  public double getMaxDegrees() {
    return Units.radiansToDegrees(maxRadians);
  }

  public double getActualDegrees() {
    return Units.radiansToDegrees(m_driveEncoder.getPosition());
  }

  public double getTargetDegrees() {
    return Units.radiansToDegrees(m_targetRadians);
  }

  public void driveArm(double degrees) {
    if (Math.abs(degrees) > 0.05) {
      m_targetRadians += Units.degreesToRadians(degrees * 10);
    }
    setReferencePeriodic();
  }

  private void setReferencePeriodic() {
    m_targetRadians = MathUtil.clamp(m_targetRadians, minRadians, maxRadians);
    m_driveController.setReference(m_targetRadians, ControlType.kPosition);
  }
}