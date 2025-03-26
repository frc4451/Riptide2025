package frc.robot.subsystems.quest;

public class QuestIOReal implements QuestIO {
  private final QuestNav questNav;

  public QuestIOReal() {
    questNav = new QuestNav(() -> QuestConstants.robotToQuestTransform);
    questNav.zeroPosition();
    questNav.zeroHeading();
  }

  /** Update inputs supplied */
  public void updateInputs(QuestIOInputs inputs) {
    inputs.rawPose = questNav.getRawPose();
    inputs.pose = questNav.getPose();
    inputs.questPose = questNav.getQuestNavPose();
    inputs.resetRobotPose = questNav.getResetPose();
    inputs.connected = questNav.connected();
    inputs.batteryLevel = questNav.getBatteryPercent();
    inputs.timestamp = questNav.timestamp();

    questNav.processHeartbeat();
    questNav.cleanUpQuestNavMessages();
  }

  @Override
  public void close() {
    questNav.close();
  }
}
