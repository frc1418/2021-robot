package frc.robot.subsystems;

import static org.mockito.Mockito.verify;

import frc.robot.common.Odometry;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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

    @Test
    public void testArcadeDrive() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain, leftMotors, rightMotors, odometry);
        driveSubsystem.drive(1, 0);
        verify(driveTrain).arcadeDrive(1, 0);
    }
}
