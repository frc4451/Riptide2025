package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean connected = false;

    // public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

    public Translation2d[] validCorners = new Translation2d[0];
    public Translation2d[] rejectedCorners = new Translation2d[0];

    public int[] validIds = new int[0];
    public int[] rejectedIds = new int[0];

    public Pose3d[] validAprilTagPoses = new Pose3d[0];
    public Pose3d[] rejectedAprilTagPoses = new Pose3d[0];

    public PoseObservation[] validLocalPoseObservations = new PoseObservation[0];
    public PoseObservation[] rejectedLocalPoseObservations = new PoseObservation[0];

    public PoseObservation[] validPoseObservations = new PoseObservation[0];
    public PoseObservation[] rejectedPoseObservations = new PoseObservation[0];

    public Pose3d[] validPoses = new Pose3d[0];
    public Pose3d[] rejectedPoses = new Pose3d[0];
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}

  public default void addHeadingData(double timestampSeconds, Rotation2d heading) {}
}
