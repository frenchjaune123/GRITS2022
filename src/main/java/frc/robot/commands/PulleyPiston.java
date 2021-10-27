// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pulley;
import frc.robot.subsystems.ShooterSubsystem;

public class PulleyPiston extends CommandBase {

  private final Pulley m_pulley;

  /** Creates a new ShooterPistons. */
  public PulleyPiston(Pulley pulley) {
    m_pulley = pulley;

    addRequirements(m_pulley);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pulley.push();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
