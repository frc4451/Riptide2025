package frc.robot.subsystems.quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class QuestConstants {
  public static final Transform2d robotToQuestTransform =
      new Transform2d(
          Units.inchesToMeters(-2.274634152842048),
          Units.inchesToMeters(7.481121673003204),
          Rotation2d.kCCW_90deg);

  public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);

  /**
   * "Tolerance" for how far QuestNav should travel relative to the previous
   * known Global Pose. This number will need to be tuned. However, we can guess that with
   * the following math we should find the maximum wheel odometry travel distance and use
   * that as our starting point.
   * 
   * maxSpeedMetersPerSec (4.4) / 1000 * 20 -> (0.088 meters / 3.465 inches) /robot loop.
   *
   * Make sure to compensate for other robots pushing around.
   * 
   * This an incredibly hacky solution for checking if the headset somehow 
   * "teleports off the field" again.
   */
  public static final double acceptableDistanceTolerance = Units.inchesToMeters(4);
}
