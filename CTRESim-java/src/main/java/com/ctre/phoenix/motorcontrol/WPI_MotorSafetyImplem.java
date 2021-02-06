package com.ctre.phoenix.motorcontrol;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SpeedController;

public class WPI_MotorSafetyImplem extends MotorSafety {
  private SpeedController _speedController = null;
  
  private String _description = null;
  
  public WPI_MotorSafetyImplem(SpeedController speedController, String description) {
    this._speedController = speedController;
    this._description = description;
  }
  
  public void stopMotor() {
    this._speedController.stopMotor();
  }
  
  public String getDescription() {
    return this._description;
  }
}
