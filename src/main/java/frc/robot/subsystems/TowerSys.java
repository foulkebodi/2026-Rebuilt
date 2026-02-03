// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import javax.crypto.KEMSpi;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TowerConstants;

public class TowerSys extends SubsystemBase {
  private final SparkFlex towerMtr;

  private final DigitalInput beamBreak;
  private boolean beamBreakBroken;
  private boolean shooting;
  private boolean intaking;

  /** Creates a new ExampleSubsystem. */
  public TowerSys() {

    towerMtr = new SparkFlex(CANDevices.towerMtrID, MotorType.kBrushless);
    SparkFlexConfig towerSparkFlexConfig = new SparkFlexConfig();

    towerSparkFlexConfig.inverted(true);

    towerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    towerSparkFlexConfig.smartCurrentLimit(TowerConstants.maxTowerCurrentAmps);

    towerSparkFlexConfig.voltageCompensation(9);

    towerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    towerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    towerMtr.configure(
        towerSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    beamBreak = new DigitalInput(CANDevices.beamBreakPort);

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   * 
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (beamBreakBroken && !shooting) {
      towerMtr.set(0.0);
    } else if (shooting) {
      towerMtr.set(TowerConstants.towerShootingSpeed);
    } else if (intaking) {
      towerMtr.set(TowerConstants.towerIntakingSpeed);
    } else {
      towerMtr.set(0.0);
    }
  }

  public void setShooting(boolean isShooting) {
    shooting = isShooting;
  }

  public void setIntaking(boolean isIntaking) {
    intaking = isIntaking;
  }

  public boolean getbeamBreakBroken() {
    return beamBreak.get();
  }
}