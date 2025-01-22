package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public boolean connected = false;

    // public PhotonPipelineResult[] results = new PhotonPipelineResult[0];

    public PoseObservation[] poseObservations = new PoseObservation[0];
    // public PhotonTrackedTarget[] targets = new PhotonTrackedTarget[0];
    public Translation2d[] corners = new Translation2d[0];

    // public int[] visibleIds = new int[0];
    // public int[] rejectedIds = new int[0];
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
