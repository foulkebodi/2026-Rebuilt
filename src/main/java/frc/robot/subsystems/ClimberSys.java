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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TurretConstants;

public class ClimberSys extends SubsystemBase {

  private final SparkFlex HookMtr;
  private final SparkFlex LeftElevatorMtr;
  private final SparkFlex RightElevatorMtr;

  private final RelativeEncoder LeftElevatorEnc;
  private final RelativeEncoder RightElevatorEnc;
  private final RelativeEncoder HookEnc;

  private final SparkClosedLoopController HookPID;
  //private final SparkClosedLoopController LeftElevatorPID;
  //private final SparkClosedLoopController RightElevatorPID;

  private final ProfiledPIDController LeftElevatorPID;
  private final ProfiledPIDController RightElevatorPID;

  public enum ElevatorState {
    HOME,
    L1,
    L1_HANDOFF,
    L1_BUFFER,
    L2,
    L2_HANDOFF,
    L2_BUFFER,
    L3,
    L3_HANDOFF,
    L3_BUFFER
  }

  private ElevatorState currentState = ElevatorState.HOME;

  private double targetElevatorPosition;

  @SuppressWarnings("removal")
  public ClimberSys() {
    HookMtr = new SparkFlex(CANDevices.HookMtrID, MotorType.kBrushless);
    SparkFlexConfig HookMtrSparkFlexConfig = new SparkFlexConfig();
    HookPID = HookMtr.getClosedLoopController();
    HookEnc = HookMtr.getEncoder();

    LeftElevatorMtr = new SparkFlex(CANDevices.LeftElevatorID, MotorType.kBrushless);
    SparkFlexConfig LeftElevatorMtrSparkFlexConfig = new SparkFlexConfig();
   // LeftElevatorPID = LeftElevatorMtr.getClosedLoopController();
    LeftElevatorPID = new ProfiledPIDController(
      ClimberConstants.ElevatorP, 0.0, ClimberConstants.ElevatorD,
      new TrapezoidProfile.Constraints(ClimberConstants.ElevatorMaxVelocityInchesPerSecond,
      ClimberConstants.ElevatorMaxAccelerationInchesPerSecond));
    LeftElevatorEnc = LeftElevatorMtr.getEncoder();

    RightElevatorMtr = new SparkFlex(CANDevices.RightElevatorID, MotorType.kBrushless);
    SparkFlexConfig RightElevatorMtrSparkFlexConfig = new SparkFlexConfig();
    //RightElevatorPID = RightElevatorMtr.getClosedLoopController();
    RightElevatorPID = new ProfiledPIDController(
      ClimberConstants.ElevatorP, 0.0, ClimberConstants.ElevatorD,
      new TrapezoidProfile.Constraints(ClimberConstants.ElevatorMaxVelocityInchesPerSecond,
      ClimberConstants.ElevatorMaxAccelerationInchesPerSecond));
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
    LeftElevatorMtrSparkFlexConfig.voltageCompensation(12);
    RightElevatorMtrSparkFlexConfig.voltageCompensation(12);

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
    LeftElevatorMtr.set(LeftElevatorPID.calculate(getClimberPosition()));
    RightElevatorMtr.set(RightElevatorPID.calculate(getClimberPosition()));
    
    // currently the control remains within the following methods, but this
    // can be changed to allow for more complex control logic if desired
  }

  // public void setState(ElevatorState newState) {
  //   currentState = newState;
  //   switch (newState) {
  //     case HOME -> setClimberPosition(0);
  //     case L1 -> setClimberPosition(Constants.ClimberConstants.ClimberL1Position);
  //     case L1_HANDOFF -> setClimberPosition(Constants.ClimberConstants.ClimberL1HandoffPosition);
  //     case L1_BUFFER -> setClimberPosition(Constants.ClimberConstants.ClimberL1BufferPosition);
  //     case L2 -> setClimberPosition(Constants.ClimberConstants.ClimberL2Position);
  //     case L2_HANDOFF -> setClimberPosition(Constants.ClimberConstants.ClimberL2HandoffPosition);
  //     case L2_BUFFER -> setClimberPosition(Constants.ClimberConstants.ClimberL2BufferPosition);
  //     case L3 -> setClimberPosition(Constants.ClimberConstants.ClimberL3Position);
  //     case L3_HANDOFF -> setClimberPosition(Constants.ClimberConstants.ClimberL3HandoffPosition);
  //     case L3_BUFFER -> setClimberPosition(Constants.ClimberConstants.ClimberL3BufferPosition);
  //   }
  // }

  // public void incrementState() {
  //   ElevatorState[] states = ElevatorState.values();
  //   int nextIndex = currentState.ordinal() + 1;
  //   if (nextIndex >= states.length) {
  //     nextIndex = states.length - 1;
  //   }
  //   setState(states[nextIndex]);
  // }

  // public void decrementState() {
  //   ElevatorState[] states = ElevatorState.values();
  //   int prevIndex = currentState.ordinal() - 1;
  //   if (prevIndex < 0) {
  //     prevIndex = 0;
  //   }
  //   setState(states[prevIndex]);
  // }

  public ElevatorState getCurrentState() {
    return currentState;
  }

  // public void setClimberPosition(double targetPos) {
  //   LeftElevatorPID.setSetpoint(targetPos, ControlType.kPosition);
  //   RightElevatorPID.setSetpoint(targetPos, ControlType.kPosition);
  // }
  public void setClimberPosition(double targetpos){
   LeftElevatorPID.setGoal(targetpos);
   RightElevatorPID.setGoal(targetpos);
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

  // public void manualClimberPosition(double speed) {
  // LeftElevatorMtr.set(speed);
  // RightElevatorMtr.set(speed);
  // }
}
