package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CommandCustomXboxController extends CommandXboxController {
  /**
   * Construct an instance of a command controller.
   *
   * @param port The port on Driver Station where the controller is connected.
   */
  public CommandCustomXboxController(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    return applyJoystickDeadband(super.getLeftX());
  }

  public double getLeftXSquared() {
    return squareJoystickValue(getLeftX());
  }

  @Override
  public double getLeftY() {
    return applyJoystickDeadband(super.getLeftY());
  }

  public double getLeftYSquared() {
    return squareJoystickValue(getLeftY());
  }

  @Override
  public double getRightX() {
    return applyJoystickDeadband(super.getRightX());
  }

  public double getRightXSquared() {
    return squareJoystickValue(getRightX());
  }

  @Override
  public double getRightY() {
    return applyJoystickDeadband(super.getRightY());
  }

  public double getRightYSquared() {
    return squareJoystickValue(getRightY());
  }

  /**
   * @return Trigger instance that is true when the Left Stick's X is not 0.
   */
  public Trigger leftX() {
    return new Trigger(() -> getLeftX() != 0);
  }

  /**
   * @return Trigger instance that is true when the Left Stick's Y is not 0.
   */
  public Trigger leftY() {
    return new Trigger(() -> getLeftY() != 0);
  }

  /**
   * @return Trigger instance that is true when the Right Stick's X is not 0.
   */
  public Trigger rightX() {
    return new Trigger(() -> getRightX() != 0);
  }

  /**
   * @return Trigger instance that is true when the Right Stick's Y is not 0.
   */
  public Trigger rightY() {
    return new Trigger(() -> getRightY() != 0);
  }

  /**
   * NOTE: Trigger Treshold has been overriden by {@link CommandCustomXboxController}, check see
   * block below.<br>
   * ORIGINAL DOCS: {@inheritDoc}
   *
   * @see #triggerPressedThreshold
   */
  @Override
  public Trigger leftTrigger() {
    return leftTrigger(Constants.triggerPressedThreshold);
  }

  /**
   * NOTE: Trigger Treshold has been overriden by {@link CommandCustomXboxController}, check see
   * block below.<br>
   * ORIGINAL DOCS: {@inheritDoc}
   *
   * @see #triggerPressedThreshold
   */
  @Override
  public Trigger rightTrigger() {
    return rightTrigger(Constants.triggerPressedThreshold);
  }

  public Command rumble(double strength) {
    return Commands.startEnd(
        () -> setRumble(RumbleType.kBothRumble, strength),
        () -> setRumble(RumbleType.kBothRumble, 0));
  }

  public Command rumbleSeconds(double strength, double time) {
    return rumble(strength).withTimeout(time);
  }

  public Command rumbleOnOff(double strength, double rumbleTime, double waitTime, int loops) {
    return Commands.repeatingSequence(
            rumbleSeconds(strength, rumbleTime), Commands.waitSeconds(waitTime))
        .withTimeout((rumbleTime + waitTime) * loops);
  }

  private double squareJoystickValue(double value) {
    return Math.abs(value) * value;
  }

  /**
   * Applies a predefined deadband to a value. Meant for joysticks.
   *
   * @param stickValue Value of a joystick, usually [-1.0, 1.0]
   * @return Joystick's value with a deadband applied
   */
  private double applyJoystickDeadband(double stickValue) {
    return MathUtil.applyDeadband(stickValue, Constants.controllerDeadband);
  }
}
