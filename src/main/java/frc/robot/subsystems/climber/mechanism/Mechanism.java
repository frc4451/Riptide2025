package frc.robot.subsystems.climber.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.climber.ClimberConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Mechanism {
  private final LoggedMechanism2d mechanism;

  private final LoggedMechanismLigament2d pivot;

  private final String logRoot;

  public Mechanism(String logRoot, Color color, double lineWidth) {
    this.logRoot = logRoot;
    mechanism =
        new LoggedMechanism2d(
            MechanismConstants.displayWidth,
            MechanismConstants.displayHeight,
            new Color8Bit(Color.kWhite));

    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            "root", MechanismConstants.rootPosition.getX(), MechanismConstants.rootPosition.getY());

    pivot =
        new LoggedMechanismLigament2d(
            "pivot",
            MechanismConstants.pivotLength,
            ClimberConstants.intialPosition.getDegrees(),
            lineWidth,
            new Color8Bit(color));
    root.append(pivot);
  }

  public void update(Rotation2d angle) {
    pivot.setAngle(MechanismConstants.pivotInitialAngle.plus(angle));

    Logger.recordOutput(logRoot + "/Mechanism2d", mechanism);
  }
}
