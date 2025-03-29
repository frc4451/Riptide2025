package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;

public enum Cages {
  OUTSIDE(new Pose2d(), new Pose2d()),
  MIDDLE(new Pose2d(), new Pose2d()),
  INNER(new Pose2d(), new Pose2d()),
  ;

  public Pose2d blue;
  public Pose2d red;

  private Cages(Pose2d blue, Pose2d red) {
    this.blue = blue;
    this.red = red;
  }

  public Pose2d get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
