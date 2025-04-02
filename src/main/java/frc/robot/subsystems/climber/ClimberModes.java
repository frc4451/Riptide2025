package frc.robot.subsystems.climber;

public enum ClimberModes {
  TUCK(0.0, 0.0, 0.0),
  EXTEND(-13.5, 0.1, 0.1);

  public final double positionInches;
  public final double hookPosition;
  public final double trayPosition;

  private ClimberModes(double positionInches, double hookPosition, double trayPosition) {
    this.positionInches = positionInches;
    this.hookPosition = hookPosition;
    this.trayPosition = trayPosition;
  }
}
