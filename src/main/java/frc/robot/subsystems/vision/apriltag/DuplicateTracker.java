package frc.robot.subsystems.vision.apriltag;

import org.photonvision.targeting.PhotonPipelineResult;

public class DuplicateTracker {
  private double lastTimestamp;

  /**
   * Compares the previous timestamp to the current timestamp. If the current timestamp matches the
   * previous timestamp, that indicates that current frame is a duplicate.
   *
   * <p>Duplicate Frames should be disregarded.
   *
   * @param frame - Current PhotonVision Pipeline Result
   * @return whether the current frame was the last frame we received (is a duplicate)
   */
  public boolean isDuplicateFrame(PhotonPipelineResult frame) {
    double timestamp = frame.getTimestampSeconds();
    boolean isDuplicate = lastTimestamp == timestamp;
    this.lastTimestamp = timestamp;
    return isDuplicate;
  }
}
