package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.bobot_state.BobotState;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class QuestSubsystem extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

  public QuestSubsystem(QuestIO io) {
    this.io = io;
    io.resetPose(Pose2d.kZero);
    io.zeroAbsolutePosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    disconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
    lowBatteryAlert.set(inputs.batteryLevel < 25 && !disconnectedAlert.get());

    if (DriverStation.isEnabled() && inputs.connected) {
      BobotState.offerQuestMeasurment(new TimestampedPose(inputs.robotPose, inputs.timestamp));
    } else {
      io.resetPose(BobotState.getGlobalPose());
    }
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public void zeroAbsolutePosition() {
    io.zeroAbsolutePosition();
  }

  @Override
  public void simulationPeriodic() {}
}
