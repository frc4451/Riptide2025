package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberConstants {
  public static final int canId = 8;

  public static final DCMotor gearbox = DCMotor.getFalcon500Foc(1);

  public static final double circumferenceOfSpool = 1.0 * Math.PI;
  public static final double reduction = 25.0;

  public static final double moi = 1.0;

  public static final double maxPositionInches = 3.5;

  public static final boolean invert = true;
  public static final double currentLimitAmps = 50.0;

  public static final boolean isBrakeMode = true;

  public static final Rotation2d intialPosition = Rotation2d.fromDegrees(90);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
  public static final Slot0Configs gains =
      new Slot0Configs()
          // feedforward
          .withKG(0.0)
          .withKS(0.0)
          .withKV(0.1)
          .withKA(0.0)
          // feedback
          .withKP(1.0)
          .withKI(0.0)
          .withKD(0.0);

  public static final MotionMagicConfigs mmConfig =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(10 / circumferenceOfSpool * reduction)
          .withMotionMagicAcceleration(25 / circumferenceOfSpool * reduction);

  public static final boolean foc = true;

  public static final int hookServoChannel = 9;
  public static final double hookServoDeployPosition = 0.01;

  public static final int trayServoChannel = 7;
  public static final double trayServoDeployPosition = -0.35;
}
