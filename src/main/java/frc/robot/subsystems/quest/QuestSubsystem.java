package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.VirtualSubsystem;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.Logger;

public class QuestSubsystem extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Queue<TimestampedPose> measurements = new ArrayBlockingQueue<>(20);

  private final Alert lowBatteryAlert = new Alert("Low Alert", AlertType.kWarning);

  public QuestSubsystem(QuestIO io) {
    this.io = io;
    io.resetPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {
    final String logRoot = "Oculus/";

    io.updateInputs(inputs);
    Logger.processInputs(logRoot, inputs);

    lowBatteryAlert.set(inputs.batteryLevel < 25);

    measurements.add(new TimestampedPose(inputs.pose, inputs.timestamp));
  }

  @Override
  public void simulationPeriodic() {}
}
