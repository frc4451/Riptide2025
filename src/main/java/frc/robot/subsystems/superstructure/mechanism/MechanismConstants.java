package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class MechanismConstants {
  public static final double displayWidth = 100.0;
  public static final double displayHeight = 200.0;

  public static final double lineWidth = 6.0;

  public static final double elevatorInitialHeight = 35.0;
  public static final Rotation2d elevatorRotation = Rotation2d.kCCW_Pi_2;

  public static final double coralPivotLength = 26.0;
  public static final Rotation2d coralInitialAngle = Rotation2d.fromDegrees(-45.0);
  public static final double coralElevatorOffset = -21.0;

  public static final double algaePivotLength = 7.0;
  public static final Rotation2d algaeInitialAngle = Rotation2d.fromDegrees(180.0);
  public static final double algaeElevatorOffset = 0.0;
  public static final Translation2d rootPosition = new Translation2d(25.0, 25.0);
}
