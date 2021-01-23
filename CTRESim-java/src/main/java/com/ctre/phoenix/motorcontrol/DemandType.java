package com.ctre.phoenix.motorcontrol;

public enum DemandType {
  Neutral(0),
  AuxPID(1),
  ArbitraryFeedForward(2);
  
  public int value;
  
  DemandType(int value) {
    this.value = value;
  }
}