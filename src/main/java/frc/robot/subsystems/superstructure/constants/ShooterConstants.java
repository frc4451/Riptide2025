package frc.robot.subsystems.superstructure.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterConstants {
  public static final int canId = 4;
  public static final int coralSensorId = 1;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);
  public static final double reduction = 1.0;
  public static final double moi = 0.001;

  public static final boolean invert = false;
  public static final double currentLimitAmps = 20.0;
}
