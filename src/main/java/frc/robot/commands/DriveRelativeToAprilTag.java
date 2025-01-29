package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;

public class DriveRelativeToAprilTag {
    public Command driveToOffsetCommand(Drive drive, int tagId, Supplier<Double> xOffset, Supplier<Double> yOffset, Supplier<Rotation2d> thetaOffset) {
        PIDController xController = new PIDController(0.0, 0.0, 0.0);
        PIDController yController = new PIDController(0.0, 0.0, 0.0);  
        PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

        Transform2d offset = new Transform2d(xOffset.get(), yOffset.get(), thetaOffset.get());

        return Commands.run(() -> {
            Pose2d tagPose = VisionConstants.fieldLayout.getTagPose(tagId).get().toPose2d(); // Need to change this to the relative position calc'ed by the camera
    
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
        }, drive);
    }

    public Command driveParallelCommand(Drive drive, int tagId, Supplier<Distance> distance) {
        Transform2d robotToTagOffset = null; // Fix this

        Rotation2d desiredTheta = robotToTagOffset.getRotation();

        // Project Robot x and y onto the offset to get distance error to the line
        // double desiredX = 
        return Commands.none();
    }
}