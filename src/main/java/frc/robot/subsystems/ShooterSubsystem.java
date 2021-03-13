package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private static final double BALL_EXISTS_DISTANCE = 3.5; // Inches
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final Solenoid shooterSolenoid;
    private Ultrasonic ballSensor;
    private final CANPIDController shooterController;
    private final CANEncoder shooterEncoder;
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/launcher");
    private final NetworkTableEntry rpm = table.getEntry("filtered_rpm");
    // private final NetworkTableEntry ff = table.getEntry("ff");
    // private final NetworkTableEntry p = table.getEntry("p");
    private final NetworkTableEntry output = table.getEntry("output");
    private final NetworkTableEntry current = table.getEntry("current");
    // private final MedianFilter rangeFilter = new MedianFilter(3);
    private double targetRPM = 0;

    public ShooterSubsystem(
            CANSparkMax shooterMotor1,
            CANSparkMax shooterMotor2,
            Solenoid shooterSolenoid,
            CANEncoder encoder,
            Ultrasonic ballSensor) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.shooterSolenoid = shooterSolenoid;
        this.shooterController = shooterMotor1.getPIDController();
        this.shooterEncoder = encoder;

        //P: 0.0002
        //D: 0.0001
        //F: 0.00018
        this.shooterController.setFF(0.00018);
        this.shooterController.setP(0.0004);
        this.shooterController.setD(0.00011);
        /* p.setDefaultDouble(0);
        p.addListener(
                notification -> {
                    this.shooterController.setP(notification.value.getDouble());
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        ff.setDefaultDouble(0);
        ff.addListener(
                notification -> {
                    this.shooterController.setFF(notification.value.getDouble());
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate); */

        Ultrasonic.setAutomaticMode(true);
        ballSensor.setEnabled(true);
        this.ballSensor = ballSensor;

        shooterMotor1.setInverted(true);
        shooterMotor1.setClosedLoopRampRate(1);
        shooterMotor1.setOpenLoopRampRate(0);
        shooterMotor2.follow(shooterMotor1, true);
    }

    public boolean isBallReady() {
        return ballSensor.getRangeInches() <= BALL_EXISTS_DISTANCE;
    }

    public void activatePiston() {
        shooterSolenoid.set(true);
        // rangeFilter.reset();
    }

    public void lowerPiston() {
        shooterSolenoid.set(false);
    }

    public void shootVelocity(double shooterSpeed) {
        shooterController.setReference(shooterSpeed, ControlType.kVelocity);
        targetRPM = shooterSpeed;
    }

    public void shootVoltage(double shooterSpeed) {
        shooterMotor1.set(shooterSpeed);
    }

    public void periodic() {
        rpm.setDouble(this.shooterEncoder.getVelocity());
        output.setDouble(this.shooterMotor1.getAppliedOutput());
        current.setDouble(this.shooterMotor1.getOutputCurrent());
        // rangeFilter.calculate(ballSensor.getRangeInches());
    }

    public boolean isAtTargetSpeed() {
        double currentRPM = this.shooterEncoder.getVelocity();
        return (Math.abs(currentRPM - targetRPM) <= 150);
    }

    public static class Constants {
        public static final int INITITATION_LINE_VEL = 4500;
        public static final int TRENCH_LINE_VEL = 4700;
        public static final int NUM_BALLS_LOADED = 3;
    }
}
