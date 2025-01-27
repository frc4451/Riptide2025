// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.pigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationX();

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.withMountPoseYaw(DriveConstants.mountPoseYawDeg)
        .withMountPosePitch(DriveConstants.mountPosePitchDeg)
        .withMountPoseRoll(DriveConstants.mountPoseRollDeg);

    pigeon.getConfigurator().setYaw(0.0);
    pigeon.getConfigurator().apply(config);

    yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.phoenixUpdateFreqHz, yawVelocity, accelX, accelY, accelZ);

    pigeon.optimizeBusUtilization();
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY, accelZ).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.accelerationX = accelX.getValue().in(MetersPerSecondPerSecond);
    inputs.accelerationY = accelY.getValue().in(MetersPerSecondPerSecond);
    inputs.accelerationZ = accelZ.getValue().in(MetersPerSecondPerSecond);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
