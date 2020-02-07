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

    //Name: Brennan 
    //About: use the constants from the table to move the shooter to the designated angle 
    double kangle = m_shooter.hoodAngleTable();
    m_shooter.setHoodWithAngle(kangle);
  }

  @Override
  public void end(boolean interrupted) {
    
    //Name: Brennan 
    //About: reset the hood angle back tp 90 to reset the shooter 
    m_shooter.setHoodWithAngle(90);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
