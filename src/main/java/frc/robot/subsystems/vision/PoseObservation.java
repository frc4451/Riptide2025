package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimationMethod;

public record PoseObservation(
    Pose3d robotPose,
    double timestampSeconds,
    double ambiguity,
    int id,
    Matrix<N3, N1> stdDevs,
    PoseEstimationMethod method) {}
