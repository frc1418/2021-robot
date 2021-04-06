/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import frc.robot.common.Odometry;


public class DriveSubsystem extends SubsystemBase {

    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(
        DRIVE_KS,
        DRIVE_KV,
        DRIVE_KA);
    private final DifferentialDrive driveTrain;
    private final SpeedControllerGroup leftMotors;
    private final SpeedControllerGroup rightMotors;
    private final Odometry odometry;
    private final Field2d field;
    private final Timer timer;
    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    private final NetworkTable table = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry odometryPose = table.getEntry("odometryPose");
    private final NetworkTableEntry encoderPosition = table.getEntry("encoderPose");
    private final NetworkTableEntry time = table.getEntry("time");

    // private final NetworkTable table2 = ntInstance.getTable("/../../../deploy/output/Test.wpilib.json");


    public DriveSubsystem(DifferentialDrive driveTrain, SpeedControllerGroup leftMotors, SpeedControllerGroup rightMotors, Odometry odometry, Field2d field, Timer timer) {
        this.driveTrain = driveTrain;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.odometry = odometry;
        this.field = field;
        this.timer = timer;

        SmartDashboard.putData("Field", field);
    }

    public void joystickDrive(double speed, double rotation) {
        driveTrain.setDeadband(RobotDriveBase.kDefaultDeadband);
        driveTrain.arcadeDrive(speed, rotation);
    }

    public void drive(double speed, double rotation) {
        // Arcade drive without squared inputs and a deadband of 0
        driveTrain.setDeadband(0);
        driveTrain.arcadeDrive(speed, rotation, false);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometryPose.setString(odometry.getPose().toString());
        field.setRobotPose(odometry.getPose());
        odometry.update();
        // encoderPosition.setDouble(odometry.getAverageEncoderDistance());
        // time.setDouble(timer.get());
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
