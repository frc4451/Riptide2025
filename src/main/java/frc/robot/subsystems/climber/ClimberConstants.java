package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberConstants {
  public static final int canId = 8;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);

  public static final double circumference = 1.0 * Math.PI;
  public static final double length = 2.0 * Math.PI * 15.0;
  public static final double reduction = 36.0 * circumference / length;

  public static final double moi = 1.0;

  public static final boolean invert = true;
  public static final double currentLimitAmps = 30.0;

  public static final boolean isBrakeMode = true;

  public static final Rotation2d intialPosition = Rotation2d.fromDegrees(90);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  public static final Slot0Configs gains =
      new Slot0Configs()
          // feedforward
          .withKG(0.0)
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0.0)
          // feedback
          .withKP(0.0)
          .withKI(0.0)
          .withKD(0.0);

  public static final MotionMagicConfigs mmConfig =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(0)
          .withMotionMagicAcceleration(0)
          .withMotionMagicJerk(0);

  public static final boolean foc = true;
}
