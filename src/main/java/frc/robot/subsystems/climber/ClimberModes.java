package frc.robot.subsystems.climber;

public enum ClimberModes {
  TUCK(0.0),
  EXTEND(-14.0);

  public final double positionInches;

  private ClimberModes(double positionInches) {
    this.positionInches = positionInches;
  }
}
