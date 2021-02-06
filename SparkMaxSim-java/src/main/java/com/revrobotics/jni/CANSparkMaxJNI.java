/*
 * Copyright (c) 2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package com.revrobotics.jni;

import edu.wpi.first.wpilibj.RobotBase;

public class CANSparkMaxJNI {

  static {
    if (RobotBase.isReal()) {
      throw new RuntimeException("Rev simulation library cannot be used on a real robot.");
    }
  }

//CANSparkMaxLowLevel
  public static long c_SparkMax_Create(int deviceId, int motortype) { return 0l; }
  public static void c_SparkMax_Destroy(long handle) { }
  public static int c_SparkMax_GetFirmwareVersion(long handle) { return 0; }
  //public static int c_SparkMax_GetSerialNumber(long handle, int* serialNumber[3]);
  public static int c_SparkMax_GetDeviceId(long handle) { return 0; }
  public static int c_SparkMax_SetMotorType(long handle, int type) { return 0; }
  public static int c_SparkMax_GetMotorType(long handle) { return 0; }
  public static int c_SparkMax_SetPeriodicFramePeriod(long handle, int frameId, int periodMs) { return 0; }

  public static void c_SparkMax_SetControlFramePeriod(long handle, int periodMs) { };
  public static int c_SparkMax_GetControlFramePeriod(long handle) { return 0; }
  
  public static int c_SparkMax_SetEncoderPosition(long handle, float position) { return 0; }
  public static int c_SparkMax_RestoreFactoryDefaults(long handle, boolean persist) { return 0; }

  public static int c_SparkMax_SetFollow(long handle, int followerArbId, int followerCfg) { return 0; }
  public static float c_SparkMax_SafeFloat(float f) { return 0f; }
  public static void c_SparkMax_EnableExternalControl(boolean enable) { }
  public static void c_SparkMax_SetEnable(boolean enable) { }

  public static int c_SparkMax_SetpointCommand(long handle, float value, int ctrlType,
                                    int pidSlot, float arbFeedforward, int arbFFUnits) { return 0; }

//CANSparkMax
  public static int c_SparkMax_SetInverted(long handle, boolean inverted) { return 0; }
  public static boolean c_SparkMax_GetInverted(long handle) { return false; }
  public static int c_SparkMax_SetSmartCurrentLimit(long handle, int stallLimit, int freeLimit, int limitRPM) { return 0; }
  public static int c_SparkMax_GetSmartCurrentStallLimit(long handle) { return 0; }
  public static int c_SparkMax_GetSmartCurrentFreeLimit(long handle) { return 0; }
  public static int c_SparkMax_GetSmartCurrentLimitRPM(long handle) { return 0; }
  public static int c_SparkMax_SetSecondaryCurrentLimit(long handle, float limit, int chopCycles) { return 0; }
  public static float c_SparkMax_GetSecondaryCurrentLimit(long handle) { return 0f; }
  public static int c_SparkMax_GetSecondaryCurrentLimitCycles(long handle) { return 0; }
  public static int c_SparkMax_SetIdleMode(long handle, int idlemode) { return 0; }
  public static int c_SparkMax_GetIdleMode(long handle) { return 0; }
  public static int c_SparkMax_EnableVoltageCompensation(long handle, float nominalVoltage) { return 0; }
  public static float c_SparkMax_GetVoltageCompensationNominalVoltage(long handle) { return 0f; }
  public static int c_SparkMax_DisableVoltageCompensation(long handle) { return 0; }
  public static int c_SparkMax_SetOpenLoopRampRate(long handle, float rate) { return 0; }
  public static float c_SparkMax_GetOpenLoopRampRate(long handle) { return 0f; }
  public static int c_SparkMax_SetClosedLoopRampRate(long handle, float rate) { return 0; }
  public static float c_SparkMax_GetClosedLoopRampRate(long handle) { return 0f; }
  public static boolean c_SparkMax_IsFollower(long handle) { return false; }
  public static int c_SparkMax_GetFaults(long handle) { return 0; }
  public static int c_SparkMax_GetStickyFaults(long handle) { return 0; }
  public static boolean c_SparkMax_GetFault(long handle, int faultId) { return false; }
  public static boolean c_SparkMax_GetStickyFault(long handle, int faultId) { return false; }
  public static float c_SparkMax_GetBusVoltage(long handle) { return 0f; }
  public static float c_SparkMax_GetAppliedOutput(long handle) { return 0f; }
  public static float c_SparkMax_GetOutputCurrent(long handle) { return 0f; }
  public static float c_SparkMax_GetMotorTemperature(long handle) { return 0f; }
  public static int c_SparkMax_ClearFaults(long handle) { return 0; }
  public static int c_SparkMax_BurnFlash(long handle) { return 0; }
  public static int c_SparkMax_SetCANTimeout(long handle, int timeoutMs) { return 0; }
  public static int c_SparkMax_EnableSoftLimit(long handle, int dir, boolean enable) { return 0; }
  public static boolean c_SparkMax_IsSoftLimitEnabled(long handle, int dir) { return false; }
  public static int c_SparkMax_SetSoftLimit(long handle, int dir, float limit) { return 0; }
  public static float c_SparkMax_GetSoftLimit(long handle, int dir) { return 0f; }
  public static int c_SparkMax_SetSensorType(long handle, int sensorType) { return 0; }

//Digital Input
  public static int c_SparkMax_SetLimitPolarity(long handle, int sw, int polarity) { return 0; }
  public static int c_SparkMax_GetLimitPolarity(long handle, int sw) { return 0; }
  public static boolean c_SparkMax_GetLimitSwitch(long handle, int sw) { return false; }
  public static int c_SparkMax_EnableLimitSwitch(long handle, int sw, boolean enable) { return 0; }
  public static boolean c_SparkMax_IsLimitEnabled(long handle, int sw) { return false; }

//CANAnalog
  public static float c_SparkMax_GetAnalogPosition(long handle) { return 0f; }
  public static float c_SparkMax_GetAnalogVelocity(long handle) { return 0f; }
  public static float c_SparkMax_GetAnalogVoltage(long handle) { return 0f; }
  public static int c_SparkMax_SetAnalogPositionConversionFactor(long handle, float conversion) { return 0; }
  public static int c_SparkMax_SetAnalogVelocityConversionFactor(long handle, float conversion) { return 0; }
  public static float c_SparkMax_GetAnalogPositionConversionFactor(long handle) { return 0f; }
  public static float c_SparkMax_GetAnalogVelocityConversionFactor(long handle) { return 0f; }
  public static int c_SparkMax_SetAnalogInverted(long handle, boolean inverted) { return 0; }
  public static boolean c_SparkMax_GetAnalogInverted(long handle) { return false; }
  public static int c_SparkMax_SetAnalogAverageDepth(long handle, int depth) { return 0; }
  public static int c_SparkMax_GetAnalogAverageDepth(long handle) { return 0; }
  public static int c_SparkMax_SetAnalogMeasurementPeriod(long handle, int samples) { return 0; }
  public static int c_SparkMax_GetAnalogMeasurementPeriod(long handle) { return 0; }
  public static int c_SparkMax_SetAnalogMode(long handle, int mode) { return 0; }
  public static int c_SparkMax_GetAnalogMode(long handle) { return 0; }

//CANEncoder
  public static float c_SparkMax_GetEncoderPosition(long handle) { return 0f; }
  public static float c_SparkMax_GetEncoderVelocity(long handle) { return 0f; }
  public static int c_SparkMax_SetPositionConversionFactor(long handle, float conversion) { return 0; }
  public static int c_SparkMax_SetVelocityConversionFactor(long handle, float conversion) { return 0; }
  public static float c_SparkMax_GetPositionConversionFactor(long handle) { return 0f; }
  public static float c_SparkMax_GetVelocityConversionFactor(long handle) { return 0f; }
  public static int c_SparkMax_SetAverageDepth(long handle, int depth) { return 0; }
  public static int c_SparkMax_GetAverageDepth(long handle) { return 0; }
  public static int c_SparkMax_SetMeasurementPeriod(long handle, int samples) { return 0; }
  public static int c_SparkMax_GetMeasurementPeriod(long handle) { return 0; }
  public static int c_SparkMax_SetCountsPerRevolution(long handle, int counts_per_rev) { return 0; }
  public static int c_SparkMax_GetCountsPerRevolution(long handle) { return 0; }
  public static int c_SparkMax_SetEncoderInverted(long handle, boolean inverted) { return 0; }
  public static boolean c_SparkMax_GetEncoderInverted(long handle) { return false; }

// Alternate CANEncoder
  public static int c_SparkMax_SetAltEncoderPosition(long handle, float position) { return 0; }
  public static float c_SparkMax_GetAltEncoderPosition(long handle) { return 0f; }
  public static float c_SparkMax_GetAltEncoderVelocity(long handle) { return 0f; }
  public static int c_SparkMax_SetAltEncoderPositionFactor(long handle, float conversion) { return 0; }
  public static int c_SparkMax_SetAltEncoderVelocityFactor(long handle, float conversion) { return 0; }
  public static float c_SparkMax_GetAltEncoderPositionFactor(long handle) { return 0f; }
  public static float c_SparkMax_GetAltEncoderVelocityFactor(long handle) { return 0f; }
  public static int c_SparkMax_SetAltEncoderAverageDepth(long handle, int depth) { return 0; }
  public static int c_SparkMax_GetAltEncoderAverageDepth(long handle) { return 0; }
  public static int c_SparkMax_SetAltEncoderMeasurementPeriod(long handle, int samples) { return 0; }
  public static int c_SparkMax_GetAltEncoderMeasurementPeriod(long handle) { return 0; }
  public static int c_SparkMax_SetAltEncoderCountsPerRevolution(long handle, int counts_per_rev) { return 0; }
  public static int c_SparkMax_GetAltEncoderCountsPerRevolution(long handle) { return 0; }
  public static int c_SparkMax_SetAltEncoderInverted(long handle, boolean inverted) { return 0; }
  public static boolean c_SparkMax_GetAltEncoderInverted(long handle) { return false; }
  public static int c_SparkMax_SetDataPortConfig(long handle, int config) { return 0; }

//CANPIDController
  public static int c_SparkMax_SetP(long handle, int slotID, float gain) { return 0; }
  public static int c_SparkMax_SetI(long handle, int slotID, float gain) { return 0; }
  public static int c_SparkMax_SetD(long handle, int slotID, float gain) { return 0; }
  public static int c_SparkMax_SetDFilter(long handle, int slotID, float gain) { return 0; }
  public static int c_SparkMax_SetFF(long handle, int slotID, float gain) { return 0; }
  public static int c_SparkMax_SetIZone(long handle, int slotID, float IZone) { return 0; }
  public static int c_SparkMax_SetOutputRange(long handle, int slotID, float min, float max) { return 0; }
  public static float c_SparkMax_GetP(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetI(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetD(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetDFilter(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetFF(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetIZone(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetOutputMin(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetOutputMax(long handle, int slotID) { return 0f; }

  public static int c_SparkMax_SetSmartMotionMaxVelocity(long handle, int slotID, float maxVel) { return 0; }
  public static int c_SparkMax_SetSmartMotionMaxAccel(long handle, int slotID, float maxAccel) { return 0; }
  public static int c_SparkMax_SetSmartMotionMinOutputVelocity(long handle, int slotID, float minVel) { return 0; }
  public static int c_SparkMax_SetSmartMotionAccelStrategy(long handle, int slotID, int accelStrategy) { return 0; }
  public static int c_SparkMax_SetSmartMotionAllowedClosedLoopError(long handle, int slotID, float allowedError) { return 0; }
  public static float c_SparkMax_GetSmartMotionMaxVelocity(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetSmartMotionMaxAccel(long handle, int slotID) { return 0f; }
  public static float c_SparkMax_GetSmartMotionMinOutputVelocity(long handle, int slotID) { return 0f; }
  public static int c_SparkMax_GetSmartMotionAccelStrategy(long handle, int slotID) { return 0; }
  public static float c_SparkMax_GetSmartMotionAllowedClosedLoopError(long handle, int slotID) { return 0f; }

  public static int c_SparkMax_SetIMaxAccum(long handle, int slotID, float iMaxAccum) { return 0; }
  public static float c_SparkMax_GetIMaxAccum(long handle, int slotID) { return 0; }
  public static int c_SparkMax_SetIAccum(long handle, float iAccum) { return 0; }
  public static float c_SparkMax_GetIAccum(long handle) { return 0; }

  public static int c_SparkMax_SetFeedbackDevice(long handle, int sensorID) { return 0; }
  public static int c_SparkMax_SetFeedbackDeviceRange(long handle, float min, float max) { return 0; }
  public static int c_SparkMax_GetFeedbackDeviceID(long handle) { return 0; }

  public static int c_SparkMax_GetAPIMajorRevision() { return 0; }
  public static int c_SparkMax_GetAPIMinorRevision() { return 0; }
  public static int c_SparkMax_GetAPIBuildRevision() { return 0; }
  public static int c_SparkMax_GetAPIVersion() { return 0; }

  public static int c_SparkMax_GetLastError(long handle) { return 0; }
}
