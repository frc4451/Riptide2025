package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.Supplier;

public class DriveRelativeToAprilTag {
  public Command driveToOffsetCommand(
      Drive drive,
      int tagId,
      Supplier<Double> xOffset,
      Supplier<Double> yOffset,
      Supplier<Rotation2d> thetaOffset) {
    PIDController xController = new PIDController(0.0, 0.0, 0.0);
    PIDController yController = new PIDController(0.0, 0.0, 0.0);
    PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

    Transform2d offset = new Transform2d(xOffset.get(), yOffset.get(), thetaOffset.get());

    return Commands.run(
        () -> {
          Pose2d tagPose =
              VisionConstants.fieldLayout
                  .getTagPose(tagId)
                  .get()
                  .toPose2d(); // Need to change this to the relative position calc'ed by the camera

          Pose2d targetPose = tagPose.transformBy(offset);

          Transform2d error = BobotState.getGlobalPose().minus(targetPose);

          double xSpeed = xController.calculate(error.getX());
          double ySpeed = yController.calculate(error.getY());
          double angularSpeed = thetaController.calculate(error.getRotation().getRadians());

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  angularSpeed);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command drivePerpendicularToPoseCommand(
      Drive drive, Pose2d targetPose, Supplier<Double> perpendicularInput) {
    PIDController parallelController = new PIDController(0.0, 0.0, 0.0);
    PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

    return Commands.run(
            () -> {
              Rotation2d targetRotation = targetPose.getRotation();
              //   Rotation Matrix
              CoordinateSystem globalCoordinates =
                  new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.N(), CoordinateAxis.U());
              CoordinateSystem targetRelativeCoordinates =
                  new CoordinateSystem(
                      new CoordinateAxis(targetRotation.getCos(), -targetRotation.getSin(), 0),
                      new CoordinateAxis(targetRotation.getSin(), targetRotation.getCos(), 0),
                      CoordinateAxis.U());

              Rotation2d desiredTheta = targetRotation.plus(Rotation2d.kPi);

              Pose2d robotPose = BobotState.getGlobalPose();
              Translation2d robotToTargetGlobalSpace = robotPose.minus(targetPose).getTranslation();
              Translation3d robotToTargetGlobalSpace3d =
                  new Translation3d(robotToTargetGlobalSpace.getX(), robotToTargetGlobalSpace.getY(), 0.0);
              Translation2d robotToTargetRelativeSpace =
                  CoordinateSystem.convert(
                          robotToTargetGlobalSpace3d, targetRelativeCoordinates, globalCoordinates)
                      .toTranslation2d();

              double parallelError = robotToTargetRelativeSpace.getX();
              Rotation2d thetaError = robotPose.getRotation().minus(desiredTheta);

              double parallelSpeed = parallelController.calculate(parallelError);

              double xSpeed =
                  targetPose.getRotation().getCos() * parallelSpeed
                      - targetPose.getRotation().getSin() * perpendicularInput.get();
              double ySpeed =
                  targetPose.getRotation().getSin() * parallelSpeed
                      + targetPose.getRotation().getCos() * perpendicularInput.get();

              double angularSpeed = thetaController.calculate(thetaError.getRadians());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      angularSpeed);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            })
        .andThen(
            () -> {
              parallelController.close();
              thetaController.close();
            });
  }
}
