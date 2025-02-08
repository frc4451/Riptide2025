package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MechanismConstants {
  public static final double displayWidth = 40.0;
  public static final double displayHeight = 100.0;

  public static final Translation2d rootPosition = new Translation2d(20.0, 5.0);

  public static final double elevatorInitialHeight = 35.0;
  public static final Rotation2d elevatorRotation = Rotation2d.kCCW_Pi_2;

  public static final double coralPivotLength = 7.0;
  public static final double coralPositionOffset = -21.0;

  public static final double algaePivotLength = 7.0;
  public static final double algaePositionOffset = -3.0;
}
