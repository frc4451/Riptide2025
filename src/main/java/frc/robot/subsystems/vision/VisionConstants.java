package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final double ambiguityCutoff = 0.05;

  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs =
      List.of(
          // FLO
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeftOuter",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(5.5), // forward+
                          Units.inchesToMeters(7), // left+
                          Units.inchesToMeters(8)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(30)))),
              SimCameraConfig.THRIFTY_CAM),

          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontLeftInner",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(5.5), // forward+
                          Units.inchesToMeters(7), // left+
                          Units.inchesToMeters(8)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(0)))),
              SimCameraConfig.THRIFTY_CAM),

          // FRI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRightInner",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(5.5), // forward+
                          Units.inchesToMeters(-7), // left+
                          Units.inchesToMeters(8)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(0)))),
              SimCameraConfig.THRIFTY_CAM),

          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "FrontRightOuter",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(5.5), // forward+
                          Units.inchesToMeters(-7), // left+
                          Units.inchesToMeters(8)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-30)))),
              SimCameraConfig.THRIFTY_CAM));
}
