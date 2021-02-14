package frc.robot.common;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;

public class TrajectoryLoader {
    
    public TrajectoryLoader() {
        
    }
    
    private List<File> getTrajectoryFiles() {
        File deployDirectory = Filesystem.getDeployDirectory();
        File[] trajectoryFiles = deployDirectory.listFiles();

        return Arrays.asList(trajectoryFiles);
    }

    public List<Trajectory> loadTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();
        List<File> files = getTrajectoryFiles();
        for (File file : files) {
            Trajectory trajectory = new Trajectory();
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(file.toPath());
                trajectories.add(trajectory);
            }
            catch (IOException ex) {  
            }
        }
        return trajectories;
    }
}
