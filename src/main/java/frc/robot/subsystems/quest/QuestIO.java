package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO extends AutoCloseable {
  @AutoLog
  public static class QuestIOInputs {
    public boolean connected = false;

    /** Current QuestNav pose */
    public Pose2d uncorrectedPose = Pose2d.kZero;
    /** QuestNav pose when robot code started */
    public Pose2d uncorrectedResetPose = Pose2d.kZero;
    /** Transform between QuestNav current and starting pose */
    public Transform2d uncorrectedResetToQuest = Transform2d.kZero;

    public double timestamp = 0;
    public double timestampDelta = 0;
    public double batteryLevel = 0;
  }

  public default void updateInputs(QuestIOInputs inputs) {}

  @Override
  public default void close() {}
}
