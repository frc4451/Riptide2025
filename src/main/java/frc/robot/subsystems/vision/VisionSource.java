package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionSource(String name, Transform3d robotToCamera) {}
