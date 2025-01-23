package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.SimCameraProperties;

public record SimCameraConfig(Calibration calib, CalibrationError calibError, Latency latency) {
  public static record Calibration(int widthPx, int heightPx, Rotation2d fov, double fps) {}

  public static record CalibrationError(double avgErrorPx, double errorStdDevPx) {}

  public static record Latency(double avgLatencyMs, double latencyStdDevMs) {}

  public SimCameraProperties apply(SimCameraProperties props) {
    props.setCalibration(calib.widthPx, calib.heightPx, calib.fov);
    props.setFPS(calib.fps);
    props.setCalibError(calibError.avgErrorPx, calibError.errorStdDevPx);
    props.setAvgLatencyMs(latency.avgLatencyMs);
    props.setLatencyStdDevMs(latency.latencyStdDevMs);

    return props;
  }

  /** Default simulated camera configuration for a Thrify Cam * */
  public static SimCameraConfig THRIFTY_CAM_STOCK =
      new SimCameraConfig(
          new Calibration(1600, 1304, Rotation2d.fromDegrees(55), 60),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));

  public static SimCameraConfig THRIFTY_CAM_65 =
      new SimCameraConfig(
          new Calibration(1600, 1304, Rotation2d.fromDegrees(65), 60),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));
}
