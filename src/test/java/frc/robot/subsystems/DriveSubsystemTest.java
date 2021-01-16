package frc.robot.subsystems;

import static org.mockito.Mockito.verify;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class DriveSubsystemTest {

    @Mock DifferentialDrive driveTrain;

    @Test
    public void testArcadeDrive() {
        DriveSubsystem driveSubsystem = new DriveSubsystem(driveTrain);
        driveSubsystem.drive(1, 0);
        verify(driveTrain).arcadeDrive(1, 0);
    }
}
