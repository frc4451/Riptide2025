package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs {
    public PhotonPipelineResult frame = new PhotonPipelineResult();
    public boolean isDuplicateFrame = false;
    public boolean isConnected = false;
    public int heartbeat = 0;

    public EstimatedPose estimatedPose = new EstimatedPose();

    public int[] visibleIds = new int[0];
    public Pose3d[] visiblePoses = new Pose3d[0];
  }

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
