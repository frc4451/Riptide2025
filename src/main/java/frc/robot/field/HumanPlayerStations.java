package frc.robot.field;

import frc.robot.field.FieldConstants.AprilTagStruct;

public enum HumanPlayerStations {
  LEFT(FieldConstants.blueHPSDriverLeft, FieldConstants.redHPSDriverLeft),
  RIGHT(FieldConstants.blueHPSDriverRight, FieldConstants.redHPSDriverRight);

  public final HumanPlayerStation blue;
  public final HumanPlayerStation red;

  private HumanPlayerStations(AprilTagStruct blue, AprilTagStruct red) {
    this.blue = new HumanPlayerStation(blue);
    this.red = new HumanPlayerStation(red);
  }

  public HumanPlayerStation get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
