package com.ctre.phoenix.motorcontrol;

public enum InvertType {
  None(0),
  InvertMotorOutput(1),
  FollowMaster(2),
  OpposeMaster(3);
  
  public int value;
  
  InvertType(int value) {
    this.value = value;
  }
}
