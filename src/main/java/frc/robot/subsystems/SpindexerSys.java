// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.SpindexerConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class SpindexerSys extends SubsystemBase {
   private final SparkFlex spindexerMtr;

   private boolean shooting;
   private boolean intaking;

   private final TowerSys towerSys;
  
   
  @SuppressWarnings("removal")
  public SpindexerSys(TowerSys towerSys) {

    this.towerSys = towerSys;
     spindexerMtr = new SparkFlex(CANDevices.spindexerMtrID, MotorType.kBrushless);
    SparkFlexConfig spindexerSparkFlexConfig = new SparkFlexConfig();
   
    spindexerSparkFlexConfig.inverted(true);
 
    spindexerSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    spindexerSparkFlexConfig.smartCurrentLimit(SpindexerConstants.maxSpindexerCurrentAmps);
 
    spindexerSparkFlexConfig.voltageCompensation(9);
    
 
    spindexerSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    spindexerSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    spindexerMtr.configure(
      spindexerSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if(shooting) {
        spindexerMtr.set(SpindexerConstants.spindexerShootingSpeed);
    } else if (intaking && towerSys.getbeamBreakBroken()) {
        spindexerMtr.set(SpindexerConstants.spindexerAgitatingSpeed);
    } else {
        spindexerMtr.set(0.0);
       }
  }
  public void setShooting(boolean isShooting) {
    this.shooting = isShooting;
  }
  public void setIntaking(boolean isIntaking) {
    this.intaking = isIntaking;
  }
  
  }
