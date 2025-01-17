package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.dashboard.ReefTreeSelector;
import frc.robot.dashboard.ReefTreeSelector.ReefTree;
import frc.robot.subsystems.vision.VisionSubsystem.TargetWithSource;
import frc.robot.util.VirtualSubsystem;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static ReefTree currentReef = ReefTreeSelector.A;

  private static Pose2d robotPose = new Pose2d();
  private static Set<TargetWithSource> visibleAprilTags = new HashSet<>();
  private static Optional<PhotonTrackedTarget> closestObject = Optional.empty();

  public static void updateReef(ReefTree newReef) {
    BobotState.currentReef = newReef;
  }

  public static void updateClosestObject(Optional<PhotonTrackedTarget> target) {
    closestObject = target;
  }

  public static void updateRobotPose(Pose2d estimatedPosition) {
    robotPose = estimatedPosition;
  }

  public static ReefTree getReef() {
    return currentReef;
  }

  public static Pose2d getRobotPose() {
    return robotPose;
  }

  public static void updateVisibleAprilTags(Set<TargetWithSource> trackedAprilTags) {
    visibleAprilTags = trackedAprilTags;
  }

  @Override
  public void periodic() {
    // publisher.
    Logger.recordOutput(logRoot + "Value", BobotState.currentReef);
  }

  @Override
  public void simulationPeriodic() {}
}
