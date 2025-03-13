package frc.robot.subsystems.superstructure.shooter;

public enum ShooterModes {
  NONE(0.0),
  INTAKE(3.0),
  SHOOT(3.0),
  L4_SHOOT(-3.0),
  ALGAE_INTAKING(-9.0),
  ALGAE_INTAKED(
      -12.0
          * 0.037), // This is done in percentage of 12 bc cole doesn't want to do math and stole it
  // from AllredTesting
  ALGAE_SHOOT(12.0),
  ;

  public final double voltage;

  private ShooterModes(double voltage) {
    this.voltage = voltage;
  }
}
