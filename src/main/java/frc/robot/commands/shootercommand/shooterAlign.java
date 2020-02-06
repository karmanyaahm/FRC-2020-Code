/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import the subsystems
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class shooterAlign extends CommandBase {
  private final Limelight m_limelight;
  private final Shooter m_shooter;

  public shooterAlign(Limelight limelight, Shooter shooter) {
    m_limelight = limelight;
    m_shooter = shooter;

    addRequirements(m_limelight);
    addRequirements(m_shooter);
  }


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double kangle = m_shooter.hoodAngleTable();
    m_shooter.setHoodWithAngle(kangle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
