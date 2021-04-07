package frc.robot.commands.autonomous;

import static org.junit.jupiter.api.Assertions.fail;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class BarrelRacingAutonomousTest {

    @Mock DriveSubsystem driveSubsystem;
    @Mock Odometry odometry;
    @Mock Limelight limelight;
    @Mock AHRS navx;

    @Test
    public void testTrajectoryGeneration() {
        try {
            new BarrelRacing(driveSubsystem, odometry, limelight, navx);
        } catch (TrajectoryGenerationException | MalformedSplineException e) {
            fail("Trajectory generation failed");
        }
    }
}
