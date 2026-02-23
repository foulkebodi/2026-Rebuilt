// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSys;

/** An example command that uses an example subsystem. */
public class SetIntakeActuatorInches extends Command {
  
  private final IntakeSys intakeSys;
  private double targetInches;
  public SetIntakeActuatorInches(IntakeSys intakeSys, double targetInches) {
    this.intakeSys = intakeSys;
    this.targetInches = targetInches;
    // Use addRequirements()`` here to declare subsystem dependencies.
    addRequirements(intakeSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSys.setTargetActuatorInches(targetInches);
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