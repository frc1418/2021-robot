/*
 * Copyright (c) 2018-2019 REV Robotics
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

package com.revrobotics;

public interface CANSensor {

   /**
    * Get the ID of the sensor that is connected to the SparkMax through the
    * encoder port on the front of the controller (not the top port).
    *
    * @return The ID of the sensor
    */
   public abstract int getID();

   /**
    * Set the phase of the CANSensor so that it is set to be in phase with the
    * motor itself. This only works for quadrature encoders and analog sensors.
    * This will throw an error if the user tries to set the inversion of the hall
    * effect.
    * 
    * @param inverted The phase of the sensor
    * 
    * @return CANError.kOK if successful
    */
   public abstract CANError setInverted(boolean inverted);

   /**
    * Get the phase of the CANSensor. This will just return false if the user tries
    * to get the inversion of the hall effect.
    * 
    * @return The phase of the sensor
    */
   public abstract boolean getInverted();

   public static enum FeedbackSensorType {
      kNoSensor(0), kHallSensor(1), kQuadrature(2), kSensorless(3), kAnalog(4), kAltQuadrature(5);

      @SuppressWarnings("MemberName")
      public final int value;

      FeedbackSensorType(int value) {
         this.value = value;
      }

      public static FeedbackSensorType fromId(int id) {
         switch (id) {
            case 1:
               return kHallSensor;
            case 2:
               return kQuadrature;
            case 3:
               return kSensorless;
            case 4:
               return kAnalog;
            case 5:
               return kAltQuadrature;
            default:
               return kNoSensor;
         }
      }
   }
}