package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public record PoseObservation(
    Pose3d robotPose, double timestampSeconds, double ambiguity, int id, PoseStrategy strategy) {}
