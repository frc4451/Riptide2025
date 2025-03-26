package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO extends AutoCloseable {
  @AutoLog
  public static class QuestIOInputs {
    public boolean connected = false;

    // These are with relative with offsets applied (probably what you want)
    public Pose2d questPose = new Pose2d();
    public Pose2d robotPose = new Pose2d();
    public Pose2d resetRobotPose = new Pose2d();
    public Translation2d questTranslation = new Translation2d();

    public Pose2d rawPose = new Pose2d();

    public double timestamp = 0;
    public double timestampDelta = 0;
    public double batteryLevel = 0;
  }

  public default void updateInputs(QuestIOInputs inputs) {}

  /** Sets supplied pose as origin of all calculations */
  public default void resetPose(Pose2d pose) {}

  @Override
  public default void close() {}
}
