package frc.robot.subsystems;

import static org.mockito.Mockito.verify;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.common.Odometry;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class DriveSubsystemTest {

    @Mock DifferentialDrive driveTrain;
    @Mock SpeedControllerGroup leftMotors;
    @Mock SpeedControllerGroup rightMotors;
    @Mock Odometry odometry;
    @Mock Field2d field;
    @Mock Timer timer;

    @Test
    public void testArcadeDrive() {
        DriveSubsystem driveSubsystem =
                new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry, field, timer);
        driveSubsystem.drive(1, 0);
        verify(driveTrain).arcadeDrive(1, 0, false);
    }

    @Test
    public void testJoystickDrive() {
        DriveSubsystem driveSubsystem =
                new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry, field, timer);
        driveSubsystem.joystickDrive(1, 0);
        verify(driveTrain).arcadeDrive(1, 0);
    }
}
