// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

  private final SparkFlex rollerMtr;
  private final SparkFlex leftActuatorMtr;
  private final SparkFlex rightActuatorMtr;

  private final RelativeEncoder leftActuatorEnc;
  private final RelativeEncoder rightActuatorEnc;
  private final RelativeEncoder rollerEnc;

  private final SparkClosedLoopController rollerPID;
  private final SparkClosedLoopController LeftActuatorPID;
  private final SparkClosedLoopController RightActuatorPID;

  private double targetRollerRPM;

  @SuppressWarnings("removal")
  public IntakeSys() {
    rollerMtr = new SparkFlex(CANDevices.RollerMtrID, MotorType.kBrushless);
    SparkFlexConfig RollerMtrSparkFlexConfig = new SparkFlexConfig();
    rollerPID = rollerMtr.getClosedLoopController();
    rollerEnc = rollerMtr.getEncoder();

    leftActuatorMtr = new SparkFlex(CANDevices.LeftActuatorMtrID, MotorType.kBrushless);
    SparkFlexConfig LeftActuatorMtrSparkFlexConfig = new SparkFlexConfig();
    LeftActuatorPID = leftActuatorMtr.getClosedLoopController();
    leftActuatorEnc = leftActuatorMtr.getEncoder();

    rightActuatorMtr = new SparkFlex(CANDevices.RightActuatorMtrID, MotorType.kBrushless);
    SparkFlexConfig RightActuatorMtrSparkFlexConfig = new SparkFlexConfig();
    RightActuatorPID = rightActuatorMtr.getClosedLoopController();
    rightActuatorEnc = rightActuatorMtr.getEncoder();

    RollerMtrSparkFlexConfig.inverted(false);
    LeftActuatorMtrSparkFlexConfig.inverted(true);
    RightActuatorMtrSparkFlexConfig.inverted(false);

    RollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    LeftActuatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    RightActuatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    RollerMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxRollerCurrentAmps);
    LeftActuatorMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxActuatorCurrentAmps);
    RightActuatorMtrSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxActuatorCurrentAmps);

    RollerMtrSparkFlexConfig.voltageCompensation(10);
    LeftActuatorMtrSparkFlexConfig.voltageCompensation(10);
    RightActuatorMtrSparkFlexConfig.voltageCompensation(10);

    RollerMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    RollerMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    LeftActuatorMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    LeftActuatorMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    LeftActuatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(IntakeConstants.actuatorMaxPositionInches);
    LeftActuatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(IntakeConstants.actuatorMinPositionInches);

    RightActuatorMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    RightActuatorMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    RightActuatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(IntakeConstants.actuatorMaxPositionInches);
    RightActuatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(IntakeConstants.actuatorMinPositionInches);

    LeftActuatorMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.actuatorPositionConversionFactor);
    LeftActuatorMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.actuatorVelocityConversionFactor);

    RightActuatorMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.actuatorPositionConversionFactor);
    RightActuatorMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.actuatorVelocityConversionFactor);

    RollerMtrSparkFlexConfig.encoder.positionConversionFactor(IntakeConstants.rollerPositionConversionFactor);
    RollerMtrSparkFlexConfig.encoder.velocityConversionFactor(IntakeConstants.rollerVelocityConversionFactor);

    RollerMtrSparkFlexConfig.closedLoop
        .p(IntakeConstants.RollerP)
        .d(IntakeConstants.RollerD);

    RightActuatorMtrSparkFlexConfig.closedLoop
        .p(IntakeConstants.actuatorP)
        .d(IntakeConstants.actuatorD);

    LeftActuatorMtrSparkFlexConfig.closedLoop
        .p(IntakeConstants.actuatorP)
        .d(IntakeConstants.actuatorD);

    rollerMtr.configure(
        RollerMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    leftActuatorMtr.configure(
        LeftActuatorMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightActuatorMtr.configure(
        RightActuatorMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void periodic() {
    if (getActuatorPositionInches() >= IntakeConstants.actuatorSafePositionInches) {
      rollerPID.setSetpoint(targetRollerRPM, ControlType.kVelocity);
    } else {
      rollerPID.setSetpoint(0.0, ControlType.kVelocity);
    }
  }

  public void setTargetActuatorInches(double targetPositionInches) {
    LeftActuatorPID.setSetpoint(targetPositionInches, ControlType.kPosition);
    RightActuatorPID.setSetpoint(targetPositionInches, ControlType.kPosition);
  }

  public double getActuatorPositionInches() {
    return (leftActuatorEnc.getPosition() + rightActuatorEnc.getPosition()) / 2.0;
  }

  public void setTargetRollerRPM(double targetRPM) {
     this.targetRollerRPM = targetRPM;
  }

  public double getRollerRPM() {
    return rollerEnc.getVelocity();
  }

  //  public void manualAdjustActuator(double speed) {
  //   leftActuatorMtr.set(speed);
  //   rightActuatorMtr.set(speed);
  //  }
}