package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.LinkedList;
import java.util.Queue;

// From team 1732
public class RollingAveragePose2d {
  private final int windowSize;
  private final Queue<Pose2d> poses = new LinkedList<>();

  private double sumX = 0.0;
  private double sumY = 0.0;

  private double sumCos = 0.0;
  private double sumSin = 0.0;

  public RollingAveragePose2d(int windowSize) {
    this.windowSize = windowSize;
  }

  public void addPose(Pose2d pose) {
    poses.add(pose);
    sumX += pose.getX();
    sumY += pose.getY();
    sumCos += pose.getRotation().getCos();
    sumSin += pose.getRotation().getSin();

    // if we exceed window size remove oldest pose
    if (poses.size() > windowSize) {
      Pose2d removed = poses.poll();
      sumX -= removed.getX();
      sumY -= removed.getY();
      sumCos -= removed.getRotation().getCos();
      sumSin -= removed.getRotation().getSin();
    }
  }

  public Pose2d getAveragePose() {
    if (poses.isEmpty()) {
      return Pose2d.kZero;
    }
    int size = poses.size();

    double avgX = sumX / size;
    double avgY = sumY / size;

    double avgCos = sumCos / size;
    double avgSin = sumSin / size;
    double avgTheta = Math.atan2(avgSin, avgCos);

    return new Pose2d(avgX, avgY, new Rotation2d(avgTheta));
  }
}
