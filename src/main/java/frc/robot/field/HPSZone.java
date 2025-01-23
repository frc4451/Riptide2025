package frc.robot.field;

public enum HPSZone {
  DRIVER_LEFT,
  DRIVER_RIGHT;

  public Integer getAprilTag() {
    if (this == HPSZone.DRIVER_LEFT) {
      return FieldUtils.isBlueAlliance()
          ? FieldConstants.blueHPSDriverLeft
          : FieldConstants.redHPSDriverLeft;
    }
    return FieldUtils.isBlueAlliance()
        ? FieldConstants.blueHPSDriverRight
        : FieldConstants.redHPSDriverRight;
  }
}
