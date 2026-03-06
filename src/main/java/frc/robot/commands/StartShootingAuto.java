// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.turret.StartAiming;
import frc.robot.commands.turret.StartFlywheel;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.SetIntakeActuatorInches;
import frc.robot.commands.intake.SetIntakeRollerRPM;
import frc.robot.commands.spindexer.SetSpindexerRPM;
import frc.robot.commands.tower.SetTowerRPM;

/** An example command that uses an example subsystem. */
public class StartShootingAuto extends SequentialCommandGroup {

  public StartShootingAuto(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
        new StartAiming(turretSys),
        new StartFlywheel(turretSys),
        new WaitUntilCommand(() -> turretSys.isOnTarget()),
        new WaitUntilCommand(() -> turretSys.isAtSpeed()),
        new SetTowerRPM(indexerSys, IndexerConstants.towerShootingRPM),
        new SetSpindexerRPM(indexerSys, IndexerConstants.spindexerAgitatingRPM),
        new SetIntakeRollerRPM(intakeSys, IntakeConstants.agitatingRollerRPM),
        new WaitCommand(1.0),
        new SetIntakeActuatorInches(intakeSys, Constants.IntakeConstants.actuatorSafePositionInches)
    );
  }
}