package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    // change with new robot; UNIT = inches
    private static final double CAMERA_ELEVATION = 16.5;
    private static final double TARGET_ELEVATION = 98.25;
    // UNIT = degrees;
    private static final double CAMERA_ANGLE = 13.5;
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/limelight");
    private final NetworkTableEntry yaw = table.getEntry("/tx");
    private final NetworkTableEntry pitch = table.getEntry("/ty");
    private final NetworkTableEntry lightMode = table.getEntry("/ledMode");
    private final NetworkTableEntry validTarget = table.getEntry("/tv");
    private final NetworkTableEntry skew = table.getEntry("/ts");
    private final NetworkTableEntry cameraMode = table.getEntry("/camMode");
    private final NetworkTableEntry pipeline = table.getEntry("/pipeline");
    private final NetworkTableEntry targetState = table.getEntry("/target_state");
    private final NetworkTableEntry poseData = table.getEntry("/camtran");
    private Pose2d averagePose;

    /**
     * Constructor
     */
    public Limelight() {
        yaw.setDefaultDouble(0);
        pitch.setDefaultDouble(0);
        lightMode.setDefaultNumber(0);
        validTarget.setDefaultNumber(0);
        skew.setDefaultDouble(0);
        cameraMode.setDefaultNumber(0);
        pipeline.setDefaultNumber(0);
        targetState.setDefaultNumber(0);
        poseData.setDefaultDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getPose();
        if (currentPose != null) {
            averagePose = computeAveragePose(averagePose, currentPose);
        }
    }

    public double getYaw() {
        return this.yaw.getDouble(0);
    }

    public double getPitch() {
        return pitch.getDouble(0);
    }

    public int getLightMode() {
        return lightMode.getNumber(0)
            .intValue();
    }

    public void setLEDMode(LEDMode ledMode) {
        this.lightMode.setNumber(ledMode.intValue());
    }

    public boolean hasValidTarget() {
        return validTarget.getNumber(0)
            .intValue() == 1;
    }

    public double getSkew() {
        return skew.getDouble(0);
    }

    public int getCameraMode() {
        return cameraMode.getNumber(0)
            .intValue();
    }

    public void setCameraMode(int cameraMode) {
        this.cameraMode.setNumber(cameraMode);
    }

    public int getPipeline() {
        return pipeline.getNumber(0)
            .intValue();
    }

    public void setPipeline(int pipeline) {
        this.pipeline.setNumber(pipeline);
    }

    /**
     * @return The plane distance (along horizonntal plane) to the target or -1 if no target is
     * found.
     */
    public double getPlaneDistance() {
        if (!hasValidTarget()) {
            return -1;
        }

        return (TARGET_ELEVATION - CAMERA_ELEVATION) / Math.tan(
            Math.toRadians(CAMERA_ANGLE + getPitch()));
    }

    /**
     * @return The distance to the target or -1 if no target is found.
     */
    public double getDistance() {
        if (!hasValidTarget()) {
            return -1;
        }

        return (TARGET_ELEVATION - CAMERA_ELEVATION) / Math.sin(
            Math.toRadians(CAMERA_ANGLE + getPitch()));
    }

    private Pose2d getPose() {
        if (getPipeline() != Constants.PIPELINE_GET_POS) {
            throw new IllegalArgumentException(
                "The limelight must be set to the right pipeline to get run getPose()");
        }

        if (!hasValidTarget()) {
            return null;
        }

        double[] data = poseData.getDoubleArray(new double[6]);
        Rotation2d rot = Rotation2d.fromDegrees((data[4] + 180) % 360);
        // TODO: figure out what "15" is. Maybe distance of center of robot to limelight?
        double x = 15 * Math.cos(rot.getDegrees()) - data[2];
        double y = 15 * Math.cos(90 - rot.getDegrees()) + data[0];
        // Change unit from inches to meters
        return new Pose2d(x / 39.37, y / 39.37, rot);
    }

    private Pose2d computeAveragePose(Pose2d averagePose, Pose2d newPose) {
        double averageX = ((9 * averagePose.getTranslation()
            .getX()) + newPose.getTranslation()
            .getX()) / 10;
        double averageY = ((9 * averagePose.getTranslation()
            .getY()) + newPose.getTranslation()
            .getY()) / 10;
        double averageRot = ((9 * averagePose.getRotation()
            .getRadians()) + newPose.getRotation()
            .getRadians()) / 10;
        return new Pose2d(averageX, averageY, new Rotation2d(averageRot));
    }

    public Pose2d getAveragePose() {
        return averagePose;
    }

    public static class Constants {

        // LED MODES /limelight/ledMode
        public static final int LED_MODE_FROM_PIPELINE = 0;
        public static final int LED_MODE_FORCE_OFF = 1;
        public static final int LED_MODE_FORCE_BLINK = 2;
        public static final int LED_MODE_FORCE_ON = 3;

        // pipeline numbers
        public static final int PIPELINE_TELEOP = 0;
        public static final int PIPELINE_GET_POS = 1;
    }
}
