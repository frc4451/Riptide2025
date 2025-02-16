package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;

public class HPSTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget = Rotation2d.kZero;
  private double distanceMeters = 0.0;

  public void update() {
    rotationTarget = FieldUtils.getClosestHPSTag().pose().getRotation().toRotation2d();
    distanceMeters =
        BobotState.getGlobalPose()
            .getTranslation()
            .getDistance(FieldUtils.getClosestHPSTag().pose().toPose2d().getTranslation());
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }
}
