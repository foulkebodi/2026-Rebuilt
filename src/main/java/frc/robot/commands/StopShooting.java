// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.turret.StartAiming;
import frc.robot.commands.turret.StartFlywheel;
import frc.robot.commands.turret.StopAiming;
import frc.robot.commands.turret.StopFlywheel;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.spindexer.SetSpindexerRPM;
import frc.robot.commands.tower.SetTowerRPM;

/** An example command that uses an example subsystem. */
public class StopShooting extends SequentialCommandGroup {

  public StopShooting(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
        new StopAiming(turretSys),
        new StopFlywheel(turretSys),
        new SetTowerRPM(indexerSys, IndexerConstants.towerIntakingRPM),
        new SetSpindexerRPM(indexerSys, 0.0),
        new WaitCommand(1.0),
        new SetTowerRPM(indexerSys, 0.0)
    );
  }
}