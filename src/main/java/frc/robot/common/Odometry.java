package frc.robot.common;

import static frc.robot.Constants.*;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Odometry {
    private final DifferentialDriveOdometry odometry;
    private final Gyro gyro;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;

    public Odometry(
            Gyro gyro,
            DifferentialDriveOdometry odometry,
            CANEncoder leftEncoder,
            CANEncoder rightEncoder) {
        this.gyro = gyro;
        this.odometry = odometry;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;

        // this.leftEncoder.setInverted(true);

        leftEncoder.setPositionConversionFactor(DRIVE_ENCODER_CONSTANT);
        leftEncoder.setVelocityConversionFactor(DRIVE_ENCODER_CONSTANT / 60);

        rightEncoder.setPositionConversionFactor(DRIVE_ENCODER_CONSTANT);
        rightEncoder.setVelocityConversionFactor(DRIVE_ENCODER_CONSTANT / 60);

        resetEncoders();
    }

    public void update() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition());
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + -rightEncoder.getPosition()) / 2.0;
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
        return odometry.getPoseMeters();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void reset(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
}
