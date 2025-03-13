package frc.robot.subsystems.quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class QuestConstants {
  // public static final Transform2d robotToQuest =
  //     new Transform2d(Units.inchesToMeters(13.75), Units.inchesToMeters(0), new Rotation2d());

  // Initial calibration transform, uncomment before calibrating
  public static final Transform2d robotToQuest =
      new Transform2d(
          0.19054394434290312,
          0.0535715470661827,
          Rotation2d.fromDegrees(0.0) // Initialize with actual rotation of the quest in real life
          );

  public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);
}
