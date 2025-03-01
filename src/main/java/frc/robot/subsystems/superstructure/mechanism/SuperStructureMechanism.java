package frc.robot.subsystems.superstructure.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
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
  private final LoggedMechanismLigament2d coralPosition;

  private final String key;

  public SuperStructureMechanism(
      String key, Color elevatorColor, Color coralColor, double lineWidth) {
    this.key = key;
    mechanism =
        new LoggedMechanism2d(
            MechanismConstants.displayWidth,
            MechanismConstants.displayHeight,
            new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "superstructure",
            MechanismConstants.rootPosition.getX(),
            MechanismConstants.rootPosition.getY());

    elevator =
        new LoggedMechanismLigament2d(
            "elevator",
            MechanismConstants.elevatorInitialHeight,
            MechanismConstants.elevatorRotation.getDegrees(),
            lineWidth,
            new Color8Bit(elevatorColor));
    root.append(elevator);

    // This is dumb, why aren't there just regular nodes with offsets instead of rotation and
    // length?
    coralPosition =
        new LoggedMechanismLigament2d(
            "coralPosition",
            MechanismConstants.coralPositionOffset,
            0.0,
            0.0, // Hides the line
            new Color8Bit(Color.kWhite));
    elevator.append(coralPosition);
    coralPivot =
        new LoggedMechanismLigament2d(
            "coralPivot",
            MechanismConstants.coralPivotLength,
            MechanismConstants.coralPivotInitialAngle.getDegrees(),
            lineWidth,
            new Color8Bit(coralColor));
    coralPosition.append(coralPivot);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double elevatorHeightInches, Rotation2d coralAngle) {
    elevator.setLength(
        MechanismConstants.elevatorInitialHeight + Units.inchesToMeters(elevatorHeightInches));

    coralPivot.setAngle(MechanismConstants.coralPivotInitialAngle.plus(coralAngle));

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
