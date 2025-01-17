package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.GarageUtils;
import java.util.List;
import java.util.Set;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagFiltering {
  private AprilTagFiltering() {}

  /***************************************************************************
   * Process the frame and determine if we need to ignore it.
   *
   * Our criteria for ignoring a frame is either:<br>
   * 1) We don't have targets to capture<br>
   * 2) The number of targets we see exceed our max threshold.
   * 3) Targets within a frame do not meet our possible combinations<br>
   *
   * @param frame      - Current {@link PhotonPipelineResult} we process
   * @param allowedIds - AprilTag IDs we check. Check {@link VisionConstants}.
   * @return If we should ignore the frame
   */
  public static boolean shouldIgnoreFrame(PhotonPipelineResult frame, Set<Integer> allowedIds) {
    // Ignore if there are either no targets, or if we exceed
    // the number of frames we allow at one time.
    if (!frame.hasTargets() || frame.getTargets().size() > VisionConstants.MAX_FRAME_FIDS) {
      return true;
    }

    List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();

    boolean allowedCombinations = allowedIds.containsAll(ids);

    return !allowedCombinations;
    // boolean possibleCombinations =
    // VisionConstants.POSSIBLE_FRAME_FID_COMBOS.stream()
    // .anyMatch(
    // possibleFidCombo -> (possibleFidCombo.containsAll(ids) &&
    // allowedCombinations));

    // return !possibleCombinations;
  }

  /**
   * Remove targets that are too far to be reliably read.
   *
   * @param frame - Current {@link PhotonPipelineResult} we process
   */
  public static void removeTooFarTargets(PhotonPipelineResult frame) {
    frame.targets.removeIf(
        tag -> {
          double maxDistance = 6.0;
          Transform3d transform = tag.getBestCameraToTarget();
          return transform.getX() > maxDistance || transform.getY() > maxDistance;
        });
  }

  /** Get IDs we're allowed to read at the time */
  public static Set<Integer> getAllowedIDs() {
    boolean isBlueAlliance = GarageUtils.isBlueAlliance();

    boolean isAutonomous = DriverStation.isAutonomous();

    // Assuming we are in autonomous, only allow specific tags to be loaded.
    return isAutonomous
        ? isBlueAlliance ? VisionConstants.BLUE_TAG_FIDS : VisionConstants.RED_TAG_FIDS
        : VisionConstants.ALL_TAGS;
  }
}
