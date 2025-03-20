package frc.robot.field;

public enum HumanPlayerStations {
  LEFT(FieldConstants.blueHPSDriverLeft, FieldConstants.redHPSDriverLeft),
  RIGHT(FieldConstants.blueHPSDriverRight, FieldConstants.redHPSDriverRight);

  public final HPSFace blue;
  public final HPSFace red;

  private HumanPlayerStations(HPSFace blue, HPSFace red) {
    this.blue = blue;
    this.red = red;
  }

  public HPSFace get() {
    return FieldUtils.isBlueAlliance() ? blue : red;
  }
}
