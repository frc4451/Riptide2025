package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;

public class BargeTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;
  private double distanceMeters = 0;

  private boolean flipped = true;

  public void update() {
    Pose2d robot = BobotState.getGlobalPose();
    // if (!BobotState.climbMode) {
    //   flipped = !FieldUtils.onAllianceSide(robot, 0);
    // }

    Pose2d bargeTagPose = FieldUtils.getBargeTag().pose().toPose2d();
    rotationTarget = bargeTagPose.getRotation().plus(flipped ? Rotation2d.kPi : Rotation2d.kZero);
    distanceMeters = bargeTagPose.getTranslation().getDistance(robot.getTranslation());
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }
}
