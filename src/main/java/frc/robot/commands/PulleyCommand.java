// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pulley;


public class PulleyCommand extends CommandBase {
  
  private final Pulley m_pulley;
  private final DoubleSupplier m_input;
  
  /** Creates a new PulleyCommand. */
  public PulleyCommand(DoubleSupplier input, Pulley pulley) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_input = input;
    m_pulley = pulley;

    addRequirements(m_pulley);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pulley.pull(m_input.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pulley.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
