/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class Shooter extends SubsystemBase {

  //About: Instanciate the motors for the fly wheel 
  WPI_TalonFX Shooter_1 = new WPI_TalonFX(15);
  WPI_TalonFX Shooter_2 = new WPI_TalonFX(16);

  WPI_TalonSRX hoodAdjuster = new WPI_TalonSRX(20);

  //About: instanciate the doubles 
  private double velo = 0.0;

  //About: instaciate the subsystems 

  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
