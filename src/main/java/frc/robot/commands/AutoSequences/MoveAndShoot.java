// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoSequences;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoIndex;
import frc.robot.commands.AutoIndexShoot;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.BangDrive;
import frc.robot.commands.CCWArc;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class MoveAndShoot extends SequentialCommandGroup {
  
  private final DriveTrain m_driveTrain;
  private DoubleSupplier m_doubleSupplier;
  private final IndexSubsystem m_indexSubsystem;

  private final ShooterSubsystem m_shooterSubsytem;
  
  /** Creates a new MoveAndShoot. */
  public MoveAndShoot(DriveTrain driveTrain, ShooterSubsystem shooterSubsystem, IndexSubsystem indexSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_shooterSubsytem = shooterSubsystem;
    m_indexSubsystem = indexSubsystem; 
    // m_doubleSupplier = doubleSupplier;

    addCommands(
                
        new BangDrive(m_driveTrain, 0.5, 0.6),
        
        
      
        new CCWArc(m_driveTrain, 0, 0.3, 7),

        // new AutoIndex(-0.5, m_indexSubsystem),
        // new AutoShoot(0.5, m_shooterSubsytem)
      
        new AutoIndexShoot(-0.5, m_indexSubsystem, 1, m_shooterSubsytem)
      );
  }

}
