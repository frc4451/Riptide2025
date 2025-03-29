package frc.robot.subsystems.climber.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MechanismConstants {
  public static final double displayWidth = 5.0;
  public static final double displayHeight = 5.0;

  public static final Translation2d rootPosition = new Translation2d(2.5, 1.0);
  public static final double pivotLength = 1.0;
  public static final double elevatorInitialHeight = 0;
  public static Rotation2d pivotInitialAngle = Rotation2d.kZero;
}
