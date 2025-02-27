package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.vision.VisionConstants.AprilTagCameraConfig;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {
  public static record AprilTagCamera(
      AprilTagIO io,
      AprilTagIOInputsAutoLogged inputs,
      VisionSource source,
      Alert disconnectedAlert) {}

  private final List<AprilTagCamera> aprilTagCameras = new ArrayList<>();

  private static final String aprilTagLogRoot = "AprilTagCamera";

  public Vision() {
    for (AprilTagCameraConfig config : VisionConstants.aprilTagCamerasConfigs) {
      AprilTagIO io;

      switch (Constants.currentMode) {
        case REAL:
          io = new AprilTagIOPhoton(config.source());
          break;
        case SIM:
          io = new AprilTagIOPhotonSim(config.source(), config.simConfig());
          break;
        case REPLAY:
        default:
          io = new AprilTagIO() {};
          break;
      }

      Alert disconnectedAlert =
          new Alert(
              aprilTagLogRoot + " " + config.source().name() + " is disconnected!",
              AlertType.kWarning);

      aprilTagCameras.add(
          new AprilTagCamera(
              io, new AprilTagIOInputsAutoLogged(), config.source(), disconnectedAlert));
    }
  }

  @Override
  public void periodic() {
    for (AprilTagCamera cam : aprilTagCameras) {
      cam.io.addHeadingData(Timer.getFPGATimestamp(), BobotState.getGlobalPose().getRotation());
      cam.io.updateInputs(cam.inputs);
      Logger.processInputs(aprilTagLogRoot + "/" + cam.source.name(), cam.inputs);

      cam.disconnectedAlert.set(!cam.inputs.connected);

      for (PoseObservation observation : cam.inputs.validPoseObservations) {
        BobotState.offerVisionObservation(observation);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.update(BobotState.getGlobalPose()));
  }
}
