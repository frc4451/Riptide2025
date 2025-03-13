package frc.robot.field;

import frc.robot.field.FieldConstants.AprilTagStruct;

public enum HumanPlayerStations {
  LEFT(FieldConstants.blueHPSDriverLeft, FieldConstants.redHPSDriverLeft),
  RIGHT(FieldConstants.blueHPSDriverRight, FieldConstants.redHPSDriverRight);

  public final AprilTagStruct blue;
  public final AprilTagStruct red;

  private HumanPlayerStations(AprilTagStruct blue, AprilTagStruct red) {
    this.blue = blue;
    this.red = red;
  }

  public AprilTagStruct get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
