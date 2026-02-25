// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.util.ExampleSubsystem;


/** An example command that uses an example subsystem. */
public class SetIndexingRPMs extends Command {
  @SuppressWarnings({"unused", "PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexerSys indexerSys;
  private final double agitatorSpeed;
  private final double towerSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetIndexingRPMs(IndexerSys indexerSys, double agitatorSpeed, double towerSpeed) {
    this.indexerSys = indexerSys;
    this.agitatorSpeed = agitatorSpeed;
    this.towerSpeed = towerSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSys.setTargetSpindexerRPM(agitatorSpeed);
    indexerSys.setTargetTowerRPM(towerSpeed);
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