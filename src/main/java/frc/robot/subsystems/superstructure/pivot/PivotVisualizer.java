package frc.robot.subsystems.superstructure.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class PivotVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d pivot;
  private final String key;

  private final double pivotLength = Units.inchesToMeters(26);
  private final Translation2d pivotOrigin = new Translation2d(0, 0);

  public PivotVisualizer(String key, Color color) {
    this.key = key;
    this.mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d root = this.mechanism.getRoot("pivot", 1.0, 0.4);
    this.pivot =
        new LoggedMechanismLigament2d("pivot", pivotLength, 20.0, 0.6, new Color8Bit(color));

    root.append(pivot);
  }

  public void update(double angleRads) {
    pivot.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Pivot/Mechanisms/" + key + "/Mechanism2d", this.mechanism);

    Pose3d pivotPose =
        new Pose3d(
            pivotOrigin.getX(), 0.0, pivotOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Pivot/Mechanisms" + key + "/Pose3d", pivotPose);
  }
}
