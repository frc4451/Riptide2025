package frc.robot.subsystems.blinkin;

public enum BlinkinState {
  // Priority is determined by definition order,
  // e.g. CORAL_IN has precedence over DEFAULT
  CORAL_IN(BlinkinColors.SOLID_LIME, BlinkinPattern.SOLID),
  /** Let the human player know that they should throw the Coral in */
  HUMAN_PLAYER_SHOULD_THROW(BlinkinColors.SOLID_HOT_PINK, BlinkinPattern.SOLID),
  /** Let the human player know that they should prepare to throw the Coral in */
  NEAR_HUMAN_PLAYER(BlinkinColors.SOLID_LIME, BlinkinPattern.BLINK),
  DEFAULT(BlinkinColors.SOLID_BLACK, BlinkinPattern.SOLID);

  public final BlinkinColors color;
  public final BlinkinPattern pattern;

  private BlinkinState(BlinkinColors color, BlinkinPattern pattern) {
    this.color = color;
    this.pattern = pattern;
  }
}
