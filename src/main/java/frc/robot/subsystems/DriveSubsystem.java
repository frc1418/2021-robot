/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Odometry;


public class DriveSubsystem extends SubsystemBase {

    private final DifferentialDrive driveTrain;
    private final SpeedControllerGroup leftMotors;
    private final SpeedControllerGroup rightMotors;
    private final Odometry odometry;


    public DriveSubsystem(DifferentialDrive driveTrain, SpeedControllerGroup leftMotors, SpeedControllerGroup rightMotors, Odometry odometry) {
        this.driveTrain = driveTrain;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.odometry = odometry;
    }

    public void drive(double speed, double rotation) {
        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        driveTrain.feed();
    }

    public void setMaxOutput(double maxOutput) {
        driveTrain.setMaxOutput(maxOutput);
    }

}
