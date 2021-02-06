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


public class DriveSubsystem extends SubsystemBase {

    private final DifferentialDrive driveTrain;
    private final SpeedControllerGroup leftMotors;
    private final SpeedControllerGroup rightMotors;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;

    private final Gyro m_gyro = new ADXRS450_Gyro();
    private final DifferentialDriveOdometry m_odometry;


    public DriveSubsystem(DifferentialDrive driveTrain, SpeedControllerGroup leftMotors, SpeedControllerGroup rightMotors, CANEncoder leftEncoder, CANEncoder rightEncoder) {
        this.driveTrain = driveTrain;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    public void drive(double speed, double rotation) {
        driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public void periodic() {
    // Update the odometry in the periodic block
        m_odometry.update(m_gyro.getRotation2d(), leftEncoder.getPosition(),
                      rightEncoder.getPosition());
    }


    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        driveTrain.feed();
      }

    public void resetEncoders() {
        //leftEncoder.reset();
        //rightEncoder.reset();
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    public CANEncoder getLeftEncoder() {
        return leftEncoder;
      }
    
    public CANEncoder getRightEncoder() {
        return rightEncoder;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public void setMaxOutput(double maxOutput) {
        driveTrain.setMaxOutput(maxOutput);
    }
    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -m_gyro.getRate();
    }

}
