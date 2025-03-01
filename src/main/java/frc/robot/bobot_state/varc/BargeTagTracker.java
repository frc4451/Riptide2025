package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldConstants.AprilTagStruct;

public class BargeTagTracker extends TagTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;
  private double distanceMeters = 0;

  @Override
  public void update(AprilTagStruct tag) {
    Pose2d closestPose = tag.pose().toPose2d();
    rotationTarget = closestPose.getRotation().plus(Rotation2d.kPi);
    distanceMeters =
        closestPose.getTranslation().getDistance(BobotState.getGlobalPose().getTranslation());
  }

  @Override
  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  @Override
  public double getDistanceMeters() {
    return distanceMeters;
  }
}
