/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDP extends SubsystemBase {
  /**
   * Creates a new PDP.
   */
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  public PDP() {
    
  }

  public void logPDP(){
    SmartDashboard.putNumber("Total Current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("Battery Voltage", pdp.getVoltage());


  }
  public void logDriveTrainCurrents(){
    //WIP
  }
  public void logSubsystemCurrents(){
    //WIP
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logPDP();
  }
}
