package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.field.FieldConstants.AprilTagStruct;

public abstract class TagTracker {
  public abstract void update(AprilTagStruct tag);

  public abstract Rotation2d getRotationTarget();

  public abstract double getDistanceMeters();
}
