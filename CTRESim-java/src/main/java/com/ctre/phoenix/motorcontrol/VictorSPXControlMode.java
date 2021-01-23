package com.ctre.phoenix.motorcontrol;

public enum VictorSPXControlMode {
  PercentOutput(0),
  Position(1),
  Velocity(2),
  Follower(5),
  MotionProfile(6),
  MotionMagic(7),
  MotionProfileArc(10),
  Disabled(15);
  
  public final int value;
  
  VictorSPXControlMode(int initValue) {
    this.value = initValue;
  }
  
  public ControlMode toControlMode() {
    switch (this.value) {
      case 0:
        return ControlMode.PercentOutput;
      case 1:
        return ControlMode.Position;
      case 2:
        return ControlMode.Velocity;
      case 5:
        return ControlMode.Follower;
      case 6:
        return ControlMode.MotionProfile;
      case 7:
        return ControlMode.MotionMagic;
      case 10:
        return ControlMode.MotionProfileArc;
      case 15:
        return ControlMode.Disabled;
    } 
    return ControlMode.PercentOutput;
  }
}
