package frc.robot.subsystems.climber;

public enum ClimberModes {
  TUCK(0.0),
  EXTEND(-13.5);

  public final double positionInches;

  private ClimberModes(double positionInches) {
    this.positionInches = positionInches;
  }
}
