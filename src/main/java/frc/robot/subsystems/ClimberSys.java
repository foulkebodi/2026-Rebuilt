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
import frc.robot.Constants.ClimberConstants;


public class ClimberSys extends SubsystemBase {

  private final SparkFlex HookMtr;
  private final SparkFlex LeftElevatorMtr;
  private final SparkFlex RightElevatorMtr;

  private final RelativeEncoder LeftElevatorEnc;
  private final RelativeEncoder RightElevatorEnc;
  private final RelativeEncoder HookEnc;

  private final SparkClosedLoopController HookPID;
  private final SparkClosedLoopController LeftElevatorPID;
  private final SparkClosedLoopController RightElevatorPID;



  @SuppressWarnings("removal")
  public ClimberSys() {
    HookMtr = new SparkFlex(CANDevices.HookMtrID, MotorType.kBrushless);
    SparkFlexConfig HookMtrSparkFlexConfig = new SparkFlexConfig();
    HookPID = HookMtr.getClosedLoopController();
    HookEnc = HookMtr.getEncoder();

    LeftElevatorMtr = new SparkFlex(CANDevices.LeftElevatorID, MotorType.kBrushless);
    SparkFlexConfig LeftElevatorMtrSparkFlexConfig = new SparkFlexConfig();
    LeftElevatorPID = LeftElevatorMtr.getClosedLoopController();
    LeftElevatorEnc = LeftElevatorMtr.getEncoder();

    RightElevatorMtr = new SparkFlex(CANDevices.RightElevatorID, MotorType.kBrushless);
    SparkFlexConfig RightElevatorMtrSparkFlexConfig = new SparkFlexConfig();
    RightElevatorPID = RightElevatorMtr.getClosedLoopController();
    RightElevatorEnc = RightElevatorMtr.getEncoder();

    HookMtrSparkFlexConfig.inverted(false);
    LeftElevatorMtrSparkFlexConfig.inverted(true);
    RightElevatorMtrSparkFlexConfig.inverted(false);

    HookMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    LeftElevatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    RightElevatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    HookMtrSparkFlexConfig.smartCurrentLimit(ClimberConstants.maxHookCurrentAmps);
    LeftElevatorMtrSparkFlexConfig.smartCurrentLimit(ClimberConstants.maxElevatorCurrentAmps);
    RightElevatorMtrSparkFlexConfig.smartCurrentLimit(ClimberConstants.maxElevatorCurrentAmps);

    HookMtrSparkFlexConfig.voltageCompensation(10);
    LeftElevatorMtrSparkFlexConfig.voltageCompensation(10);
    RightElevatorMtrSparkFlexConfig.voltageCompensation(10);

    HookMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    HookMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    HookMtrSparkFlexConfig.softLimit.forwardSoftLimit(ClimberConstants.hookMaxDeg);
    HookMtrSparkFlexConfig.softLimit.reverseSoftLimit(ClimberConstants.hookMinDeg);

    LeftElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    LeftElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    LeftElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(ClimberConstants.ElevatorMaxInches);
    LeftElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(ClimberConstants.ElevatorMinInches);

    RightElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    RightElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    RightElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(ClimberConstants.ElevatorMaxInches);
    RightElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(ClimberConstants.ElevatorMinInches);

    LeftElevatorMtrSparkFlexConfig.encoder.positionConversionFactor(ClimberConstants.elevatorPositionConversionFactor);
    LeftElevatorMtrSparkFlexConfig.encoder.velocityConversionFactor(ClimberConstants.elevatorVelocityConversionFactor);

    RightElevatorMtrSparkFlexConfig.encoder.positionConversionFactor(ClimberConstants.elevatorPositionConversionFactor);
    RightElevatorMtrSparkFlexConfig.encoder.velocityConversionFactor(ClimberConstants.elevatorVelocityConversionFactor);

    HookMtrSparkFlexConfig.encoder.positionConversionFactor(ClimberConstants.hookPositionConversionFactor);
    HookMtrSparkFlexConfig.encoder.velocityConversionFactor(ClimberConstants.hookVelocityConversionFactor);

    HookMtrSparkFlexConfig.closedLoop
        .p(ClimberConstants.HookP)
        .d(ClimberConstants.HookD);

    RightElevatorMtrSparkFlexConfig.closedLoop
        .p(ClimberConstants.ElevatorP)
        .d(ClimberConstants.ElevatorD);

    LeftElevatorMtrSparkFlexConfig.closedLoop
        .p(ClimberConstants.ElevatorP)
        .d(ClimberConstants.ElevatorD);

    HookMtr.configure(
        HookMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    LeftElevatorMtr.configure(
        LeftElevatorMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    RightElevatorMtr.configure(
        RightElevatorMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void periodic() {
// currently the control remains within the following methods, but this 
// can be changed to allow for more complex control logic if desired
    
  }

  public void setClimberPosition(double targetPos) {
    LeftElevatorPID.setSetpoint(targetPos, ControlType.kPosition);
    RightElevatorPID.setSetpoint(targetPos, ControlType.kPosition);
  }

  public void manualClimberPosition(double speed) {
    LeftElevatorMtr.set(speed);
    RightElevatorMtr.set(speed);
  }

  public double getClimberPosition() {
    return (LeftElevatorEnc.getPosition() + RightElevatorEnc.getPosition()) / 2.0;
  }

  public void setTargetHookPosition(double targetPosition) {
    HookPID.setSetpoint(targetPosition, ControlType.kPosition);
  }

  

  public double getHookPosition() {
    return HookEnc.getPosition();
  }
}
