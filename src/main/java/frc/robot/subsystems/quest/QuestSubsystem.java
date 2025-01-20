package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.VirtualSubsystem;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.Logger;

public class QuestSubsystem extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Queue<TimestampedPose> measurements = new ArrayBlockingQueue<>(20);

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert = new Alert("Quest Low Battery!", AlertType.kWarning);

  public QuestSubsystem(QuestIO io) {
    this.io = io;
    io.resetPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    lowBatteryAlert.set(inputs.batteryLevel < 25);
    disconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);

    // measurements.add(new TimestampedPose(inputs.pose, inputs.timestamp));
  }

  @Override
  public void simulationPeriodic() {}
}
