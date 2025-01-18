package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO {
  @AutoLog
  public static class QuestIOInputs {
    // These are with relative with offsets applied (probably what you want)
    public Pose2d pose = new Pose2d();
    public double yawRad = 0;

    public double timestamp = 0;
    public double batteryLevel = 0;

    public Pose2d rawPose = new Pose2d();
    public double rawYawRad = 0;

    public float[] rawPosition = {0.0f, 0.0f, 0.0f};
    public float[] rawQuaternion = {0.0f, 0.0f, 0.0f, 0.0f};
  }

  public default void updateInputs(QuestIOInputs inputs) {}

  /** Sets supplied pose as origin of all calculations */
  public default void resetPose(Pose2d pose) {}
}
