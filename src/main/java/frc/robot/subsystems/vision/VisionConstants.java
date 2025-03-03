package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG
  }

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  private static final List<AprilTagCameraConfig> guidoConfigs =
      List.of(
          new AprilTagCameraConfig(
              new VisionSource(
                  "SillyCam",
                  new Transform3d(
                      new Translation3d(
                          6.0 / 100.0, // forward+
                          29.5 / 100.0, // left+
                          26.5 / 100.0), // up+
                      new Rotation3d(0, Units.degreesToRadians(-17.5), 0))),
              SimCameraConfig.THRIFTY_CAM_90));

  private static final List<AprilTagCameraConfig> riptideConfigs =
      List.of(
          // FLO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-6.094), // forward+
                          Units.inchesToMeters(12.728), // left+
                          Units.inchesToMeters(8.010)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-27.5), Units.degreesToRadians(30)))),
              SimCameraConfig.THRIFTY_CAM_80),
          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(6.915), // forward+
                          Units.inchesToMeters(12.421), // left+
                          Units.inchesToMeters(8.141)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-10)))),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(6.915), // forward+
                          Units.inchesToMeters(-12.421), // left+
                          Units.inchesToMeters(8.141)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(10)))),
              SimCameraConfig.THRIFTY_CAM_90),
          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "RearRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-6.094), // forward+
                          Units.inchesToMeters(-12.728), // left+
                          Units.inchesToMeters(8.010)), // up+
                      new Rotation3d(
                          0, Units.degreesToRadians(-27.5), Units.degreesToRadians(-30)))),
              SimCameraConfig.THRIFTY_CAM_80));

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs =
      Constants.isGuido ? guidoConfigs : riptideConfigs;

  public static final double ambiguityCutoff = 0.05;
  public static final double singleTagPoseCutoffMeters = 4;

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
