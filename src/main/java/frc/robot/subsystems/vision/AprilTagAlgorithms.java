package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagAlgorithms {
  public static boolean isValid(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff;
  }
}
