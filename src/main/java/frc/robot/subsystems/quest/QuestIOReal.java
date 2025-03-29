package frc.robot.subsystems.quest;

public class QuestIOReal implements QuestIO {
  private final QuestNav questNav = new QuestNav();

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    inputs.connected = questNav.connected();

    inputs.uncorrectedPose = questNav.getUncorrectedPose();
    inputs.uncorrectedResetPose = questNav.getUncorrectedResetPose();
    inputs.uncorrectedResetToQuest = inputs.uncorrectedPose.minus(inputs.uncorrectedResetPose);

    double timestamp = inputs.timestamp;
    inputs.timestamp = questNav.timestamp();
    inputs.timestampDelta = timestamp - inputs.timestamp;
    inputs.batteryLevel = questNav.getBatteryPercent();

    questNav.processHeartbeat();
    questNav.cleanUpQuestNavMessages();
  }

  @Override
  public void zeroPosition() {
    questNav.zeroPosition();
  }

  @Override
  public void zeroHeading() {
    questNav.zeroHeading();
  }

  @Override
  public void close() {
    questNav.close();
  }
}
