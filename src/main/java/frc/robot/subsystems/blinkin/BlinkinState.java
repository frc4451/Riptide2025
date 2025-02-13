package frc.robot.subsystems.blinkin;

public enum BlinkinState {
  // Priority is determined by definition order,
  // e.g. NOTE has precedence over DEFAULT
  IN_RANGE(BlinkinColors.SOLID_BLUE, BlinkinPattern.SOLID),
  PAST_OPP_WING(BlinkinColors.SOLID_VIOLET, BlinkinPattern.SOLID),
  NOTE(BlinkinColors.SOLID_GOLD, BlinkinPattern.BLINK),
  DEFAULT(BlinkinColors.SOLID_BLACK, BlinkinPattern.SOLID);

  public final BlinkinColors color;
  public final BlinkinPattern pattern;

  private BlinkinState(BlinkinColors color, BlinkinPattern pattern) {
    this.color = color;
    this.pattern = pattern;
  }
}
