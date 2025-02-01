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
  public static SimCameraConfig THRIFTY_CAM_70 =
      new SimCameraConfig(
          new Calibration(1600, 1304, Rotation2d.fromDegrees(70), 60),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));

  public static SimCameraConfig THRIFTY_CAM_80 =
      new SimCameraConfig(
          new Calibration(1600, 1304, Rotation2d.fromDegrees(80), 60),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));

  public static SimCameraConfig THRIFTY_CAM_90 =
      new SimCameraConfig(
          new Calibration(1600, 1304, Rotation2d.fromDegrees(90), 60),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));

  public static SimCameraConfig ARDUCAM_OV9281_45 =
      new SimCameraConfig(
          new Calibration(1280, 720, Rotation2d.fromDegrees(45), 90),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));
  public static SimCameraConfig ARDUCAM_OV9281_55 =
      new SimCameraConfig(
          new Calibration(1280, 720, Rotation2d.fromDegrees(55), 90),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));

  public static SimCameraConfig ARDUCAM_OV9281_65 =
      new SimCameraConfig(
          new Calibration(1280, 720, Rotation2d.fromDegrees(65), 90),
          new CalibrationError(0.25, 0.08),
          new Latency(20, 5));
}
