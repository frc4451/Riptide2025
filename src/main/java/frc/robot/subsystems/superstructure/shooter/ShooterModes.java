package frc.robot.subsystems.superstructure.shooter;

public enum ShooterModes {
  NONE(0.0),
  INTAKE(3.0),
  SHOOT(3.5),
  L4_SHOOT(-3.5),
  ALGAE_INTAKING(-9.0),
  ALGAE_INTAKED(-1.5),
  ALGAE_SHOOT(12.0),
  ;

  public final double voltage;

  private ShooterModes(double voltage) {
    this.voltage = voltage;
  }
}
