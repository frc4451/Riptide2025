package frc.robot.subsystems.superstructure.modes;

public enum ShooterModes {
  NONE(0.0, false),
  INTAKE(3.0, true),
  SHOOT(3.0, false),
  ;

  public final double voltage;
  public final boolean useCanRange;

  private ShooterModes(double voltage, boolean useCanRange) {
    this.voltage = voltage;
    this.useCanRange = useCanRange;
  }
}
