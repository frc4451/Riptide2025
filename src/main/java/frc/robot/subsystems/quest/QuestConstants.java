package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class QuestConstants {
  public static final Transform3d robotToQuest =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0),
              Units.inchesToMeters(13.187500),
              Units.inchesToMeters(5.415796)),
          new Rotation3d(0.0, 0.0, 0.0));
}
