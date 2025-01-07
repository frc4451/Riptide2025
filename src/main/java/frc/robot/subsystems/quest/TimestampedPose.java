package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;

public record TimestampedPose(Pose2d pose, double timestamp) {
}
