package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TargetAngleTracker {
  public abstract void update();

  public abstract Rotation2d getRotationTarget();

  public abstract double getDistanceMeters();
}
