package com.ctre.phoenix.motorcontrol.can;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.WPI_MotorSafetyImplem;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class WPI_VictorSPX implements SpeedController, Sendable {
  private String _description;
  
  private double _speed;
  
  private SimDevice m_simDevice;
  
  private SimDouble m_simSpeed;
  
  private SimBoolean m_simInvert;
  
  public static final double kDefaultSafetyExpiration = 0.1D;
  
  private WPI_MotorSafetyImplem _motorSafety = null;
  
  private final Object _lockMotorSaf = new Object();
  
  private double _motSafeExpiration = 0.1D;
  
  private String m_name;
  
  private String m_subsystem;
  
  public WPI_VictorSPX(int deviceNumber) {
    this.m_name = "";
    this.m_subsystem = "Ungrouped";
    HAL.report(67, deviceNumber + 1);
    this._description = "Victor SPX " + deviceNumber;
    SendableRegistry.addLW(this, "Victor SPX ", deviceNumber);
    this.m_simDevice = SimDevice.create("Victor SPX", deviceNumber);
    if (this.m_simDevice != null) {
      this.m_simSpeed = this.m_simDevice.createDouble("Motor Output", false, 0.0D);
      this.m_simInvert = this.m_simDevice.createBoolean("Inverted?", false, false);
    } 
  }
  
  /**
   * ALTERED FOR USAGE (Team 1418)
   */
  public void follow(Object masterToFollow, Object followerType) {
    return;
  }

  /**
   * ALTERED FOR USAGE (Team 1418)
   */
  public void follow(Object masterToFollow) {
    follow(masterToFollow, null);
  }

  /**
   * ALTERED FOR USAGE (Team 1418)
   */
  public void setInverted(InvertType invertType) {
    // this._invert = invertType;
  }

  public void set(double speed) {
    this._speed = speed;
    set(ControlMode.PercentOutput, this._speed);
    feed();
  }

  public void pidWrite(double output) {
    set(output);
  }
  
  public double get() {
    if (this.m_simSpeed != null)
      return this.m_simSpeed.get(); 
    return this._speed;
  }
  
  public void set(ControlMode mode, double value) {
    feed();
    simSet(mode, value);
  }
  
  @Deprecated
  public void set(ControlMode mode, double demand0, double demand1) {
    feed();
    simSet(mode, demand0);
  }
  
  public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
    feed();
    simSet(mode, demand0);
  }
  
  private void simSet(ControlMode mode, double demand0) {
    if (this.m_simSpeed != null && this.m_simInvert != null) {
      switch (mode) {
        case PercentOutput:
          this.m_simSpeed.set(demand0 * (this.m_simInvert.get() ? -1 : 1));
          return;
      } 
      this.m_simSpeed.set(0.0D);
    } 
  }
  
  public void setVoltage(double outputVolts) {
    set(outputVolts / RobotController.getBatteryVoltage());
  }
  
  public void setInverted(boolean isInverted) {
    if (this.m_simInvert != null)
      this.m_simInvert.set(isInverted); 
  }
  
  public boolean getInverted() {
    if (this.m_simInvert != null)
      return this.m_simInvert.get(); 
    return false;
  }
  
  public void disable() {
    simSet(ControlMode.PercentOutput, 0.0D);
  }
  
  public void stopMotor() {
    simSet(ControlMode.PercentOutput, 0.0D);
  }
  
  public void free() {
    SendableRegistry.remove(this);
  }
  
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Speed Controller");
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Value", this::get, this::set);
  }
  
  public String getDescription() {
    return this._description;
  }
  
  private WPI_MotorSafetyImplem GetMotorSafety() {
    if (this._motorSafety == null) {
      this._motorSafety = new WPI_MotorSafetyImplem(this, getDescription());
      this._motorSafety.setExpiration(this._motSafeExpiration);
    } 
    return this._motorSafety;
  }
  
  public void feed() {
    synchronized (this._lockMotorSaf) {
      if (this._motorSafety != null)
        GetMotorSafety().feed(); 
    } 
  }
  
  public void setExpiration(double expirationTime) {
    synchronized (this._lockMotorSaf) {
      this._motSafeExpiration = expirationTime;
      if (this._motorSafety != null)
        GetMotorSafety().setExpiration(this._motSafeExpiration); 
    } 
  }
  
  public double getExpiration() {
    synchronized (this._lockMotorSaf) {
      return this._motSafeExpiration;
    } 
  }
  
  public boolean isAlive() {
    synchronized (this._lockMotorSaf) {
      if (this._motorSafety == null)
        return true; 
      return GetMotorSafety().isAlive();
    } 
  }
  
  public void setSafetyEnabled(boolean enabled) {
    synchronized (this._lockMotorSaf) {
      if (this._motorSafety != null || enabled)
        GetMotorSafety().setSafetyEnabled(enabled); 
    } 
  }
  
  public boolean isSafetyEnabled() {
    synchronized (this._lockMotorSaf) {
      if (this._motorSafety == null)
        return false; 
      return GetMotorSafety().isSafetyEnabled();
    } 
  }
}
