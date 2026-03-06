// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IndexerConstants;

public class IndexerSys extends SubsystemBase {

  private final SparkFlex towerMtr;
  private final SparkFlex spindexerMtr;

  private final RelativeEncoder towerEnc;
  private final RelativeEncoder spindexerEnc;

  private final SparkClosedLoopController towerPID;
  private final SparkClosedLoopController spindexerPID;

  /** Creates a new ExampleSubsystem. */
  @SuppressWarnings("removal")
  public IndexerSys() {

    towerMtr = new SparkFlex(CANDevices.towerMtrID, MotorType.kBrushless);
    SparkFlexConfig towerSparkFlexConfig = new SparkFlexConfig();
    towerEnc = towerMtr.getEncoder();
    towerPID = towerMtr.getClosedLoopController();

    spindexerMtr = new SparkFlex(CANDevices.spindexerMtrID, MotorType.kBrushless);
    SparkFlexConfig spindexerSparkFlexConfig = new SparkFlexConfig();
    spindexerEnc = spindexerMtr.getEncoder();
    spindexerPID = spindexerMtr.getClosedLoopController();

    towerSparkFlexConfig.inverted(false);
    spindexerSparkFlexConfig.inverted(true);

    towerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    spindexerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    towerSparkFlexConfig.smartCurrentLimit(IndexerConstants.maxTowerCurrentAmps);
    spindexerSparkFlexConfig.smartCurrentLimit(IndexerConstants.maxSpindexerCurrentAmps);

    towerSparkFlexConfig.voltageCompensation(10);
    spindexerSparkFlexConfig.voltageCompensation(10);

    towerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    towerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    spindexerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    spindexerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    towerSparkFlexConfig.encoder.positionConversionFactor(IndexerConstants.towerPositionConversionFactor);
    towerSparkFlexConfig.encoder.velocityConversionFactor(IndexerConstants.towerVelocityConversionFactor);

    spindexerSparkFlexConfig.encoder.positionConversionFactor(IndexerConstants.spindexerPositionConversionFactor);
    spindexerSparkFlexConfig.encoder.velocityConversionFactor(IndexerConstants.spindexerVelocityConversionFactor);

    towerSparkFlexConfig.closedLoop
      .p(IndexerConstants.towerP)
      .d(IndexerConstants.towerD)
      .feedForward.kS(IndexerConstants.towerkS)
      .kV(IndexerConstants.towerkV);

    spindexerSparkFlexConfig.closedLoop
      .p(IndexerConstants.spindexerP)
      .d(IndexerConstants.spindexerD);

    towerMtr.configure(
      towerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    spindexerMtr.configure(
      spindexerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  public void setTargetSpindexerRPM(double targetSpindexerRPM) {
    // if (targetSpindexerRPM != 0.0) {
      spindexerPID.setSetpoint(targetSpindexerRPM, ControlType.kVelocity);
    // } else {
      // towerMtr.stopMotor();
    // }
  }

  public void setTargetTowerRPM(double targetTowerRPM) {
    if (targetTowerRPM != 0.0) {
      towerPID.setSetpoint(targetTowerRPM, ControlType.kVelocity);
    } else {
      towerMtr.stopMotor();
    }
  }

  public double getSpindexerRPM() {
    return spindexerEnc.getVelocity();
  }

  public double getTowerRPM() {
    return towerEnc.getVelocity();
  }
}