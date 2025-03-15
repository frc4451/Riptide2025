package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class MechanismConstants {
  public static final double displayWidth = 1;
  public static final double displayHeight = 2.5;

  public static final Translation2d rootPosition = new Translation2d(displayWidth / 2.0, 0.0);

  public static final double elevatorInitialHeight = Units.inchesToMeters(50.0);
  public static final Rotation2d elevatorRotation = Rotation2d.kCCW_Pi_2;

  public static final double coralPivotLength = Units.inchesToMeters(7.0);
  public static final double coralPositionOffset = Units.inchesToMeters(-21.0);
  public static final Rotation2d coralPivotInitialAngle = Rotation2d.fromDegrees(-90.0);

  public static final double algaePivotLength = Units.inchesToMeters(7.0);
  public static final double algaePositionOffset = Units.inchesToMeters(-3.0);
}
