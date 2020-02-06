/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class FlyWheel extends CommandBase {
  private final Shooter m_shooter;
  private final Limelight m_limelight;

  public FlyWheel(Limelight limelight, Shooter shoot) {
    m_shooter = shoot;
    m_limelight = limelight;
    addRequirements(m_shooter);
    addRequirements(m_limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //TODO: Check math after the reconfigure 
    double convertedVelocity = (m_limelight.getShooterVelocity() * 4096 / 600)/2;

    m_shooter.setShootSpeed(convertedVelocity);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
