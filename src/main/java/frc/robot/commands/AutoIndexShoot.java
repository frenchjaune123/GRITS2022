// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoIndexShoot extends CommandBase {

  private final IndexSubsystem m_indexer;
  private final double m_indexInput;
  
  private final ShooterSubsystem m_shooter;
  private final double m_shooterInput;

  /** Creates a new Index. */
  public AutoIndexShoot(double indexInput, IndexSubsystem indexer, double shooterInput, ShooterSubsystem shooter) {
    m_indexer = indexer;
    m_indexInput = indexInput;
    m_shooter = shooter;
    m_shooterInput = shooterInput;
    addRequirements(m_indexer, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.set(m_indexInput);
    m_shooter.shoot(m_shooterInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
