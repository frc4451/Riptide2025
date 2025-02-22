package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;

public class BargeTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;
  private double distanceMeters = 0;

  public void update() {
    Pose2d closestPose = FieldUtils.getBargeTag().pose().toPose2d();
    rotationTarget = closestPose.getRotation().plus(Rotation2d.kPi);
    distanceMeters =
        closestPose.getTranslation().getDistance(BobotState.getGlobalPose().getTranslation());
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }
}
