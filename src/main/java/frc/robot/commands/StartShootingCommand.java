// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.turret.StartAiming;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;

/** An example command that uses an example subsystem. */
public class StartShootingCommand extends SequentialCommandGroup {
 
  public StartShootingCommand(TurretSys turretSys, IndexerSys indexerSys, IntakeSys intakeSys) {
    super(
      new StartAiming(turretSys),
      // new StartFlywheel(turretSys),
      new WaitUntilCommand(() -> turretSys.isOnTarget())
      // new StartIndexing(indexerSys),
      // PS: possibly add a intake in command after a wait or based on a timer to change the setpoint? 
      
    );
   
  }

 
}