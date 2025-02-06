package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructureMechanism {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final LoggedMechanismLigament2d coralPivot;
  private final LoggedMechanismLigament2d algaePivot;
  private final String key;

  // This is guesswork, Allred can fill this in more accurately.
  private final double elevatorLength = Units.inchesToMeters(69);

  private final double coralPivotLength = Units.inchesToMeters(26);
  private final Translation2d coralPivotOrigin = new Translation2d(0, 0);

  private final double algaePivotLength = Units.inchesToMeters(7);
  private final Translation2d algaePivotOrgin = new Translation2d(0, 0);

  public SuperStructureMechanism(
      String key, Color elevatorColor, Color coralColor, Color algaeColor) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("superstructure", 1.0, 0.4);
    elevator =
        new LoggedMechanismLigament2d(
            "elevator", elevatorLength, 0.0, 6, new Color8Bit(elevatorColor));
    root.append(elevator);
    coralPivot =
        new LoggedMechanismLigament2d(
            "coralPivot", coralPivotLength, 20.0, 6, new Color8Bit(coralColor));
    root.append(coralPivot);
    algaePivot =
        new LoggedMechanismLigament2d(
            "algaePivot", algaePivotLength, 20.0, 6, new Color8Bit(algaeColor));
    root.append(algaePivot);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double elevatorHeightInches, Rotation2d coralAngle, Rotation2d algaeAngle) {
    // TODO: Place pivots on their spots on the elevator
    elevator.setLength(elevatorHeightInches);
    coralPivot.setAngle(coralAngle);
    algaePivot.setAngle(algaeAngle);
    Logger.recordOutput("Superstructure/" + key + "/Mechanism2d", mechanism);

    // Log 3d poses
    // Pose3d pivot =
    //     new Pose3d(
    //         coralPivotOrigin.getX(),
    //         0.0,
    //         coralPivotOrigin.getY(),
    //         new Rotation3d(0.0, -angleRads, 0.0));
    // Logger.recordOutput("Pivot/Mechanisms/" + key + "/Pose3d", pivot);
  }
}
