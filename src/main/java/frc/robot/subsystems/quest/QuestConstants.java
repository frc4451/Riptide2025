package frc.robot.subsystems.quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class QuestConstants {
  public static final Transform2d robotToQuest =
      new Transform2d(Units.inchesToMeters(13.75), Units.inchesToMeters(0), new Rotation2d());

  public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);
}
