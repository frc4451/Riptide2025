package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

public class SingleTagAlgorithms {
  public static boolean isUsable(PhotonTrackedTarget target) {
    return VisionConstants.fieldLayout.getTagPose(target.getFiducialId()).isPresent()
        && target.getPoseAmbiguity() < VisionConstants.ambiguityCutoff
        && target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
            < VisionConstants.singleTagPoseCutoffMeters;
  }
}
