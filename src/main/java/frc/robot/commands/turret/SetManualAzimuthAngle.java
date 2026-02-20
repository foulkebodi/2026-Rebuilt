// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSys;

/** An example command that uses an example subsystem. */
public class SetManualAzimuthAngle extends Command {
  
  private final TurretSys turretSys;
  private double manualAzimuthAngle;
  public SetManualAzimuthAngle(TurretSys turretSys, double manualAzimuthAngle) {
    this.turretSys = turretSys;
    this.manualAzimuthAngle = manualAzimuthAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSys.setManualAzimuthAngle(manualAzimuthAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}