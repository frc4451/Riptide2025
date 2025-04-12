package frc.robot.subsystems.superstructure.shooter;

public enum ShooterModes {
  NONE(0.0),
  INTAKE(3.0),
  L1_SHOOT(4.0),
  SHOOT(5.0),
  L4_SHOOT(-5.0),
  ALGAE_INTAKING(-9.0),
  ALGAE_INTAKED(-1.5),
  ALGAE_SHOOT(12.0),
  ;

  public final double voltage;

  private ShooterModes(double voltage) {
    this.voltage = voltage;
  }
}
