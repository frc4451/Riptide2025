package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class QuestConstants {
  public static final Transform2d robotToQuest =
      new Transform2d(Units.inchesToMeters(13.75), Units.inchesToMeters(0), new Rotation2d());
}
