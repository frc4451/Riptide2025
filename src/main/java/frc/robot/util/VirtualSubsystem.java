package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/** Subsystems that are intended to run periodically, independently of the CommandScheduler. */
public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> virtualSubsystems = new ArrayList<>();

  /**
   * Every subsystem that extends VirtualSubsystem gets added to the list of Subsystems that are
   * called every robot period.
   */
  public VirtualSubsystem() {
    virtualSubsystems.add(this);
  }

  public static void listVirtualSubsystems() {
    for (VirtualSubsystem subsystem : virtualSubsystems) {
      String message =
          String.format(
              "[robotzgarage] Connected Virtual Subsystem '%s'",
              subsystem.getClass().getSimpleName());
      System.out.println(message);
    }
  }

  /** Call {@link #periodic()} on every virtual subsystem. */
  public static void runPeriodically() {
    for (VirtualSubsystem subsystem : virtualSubsystems) {
      subsystem.periodic();
    }
  }

  /** Call {@link #simulationPeriodic()} on every virtual subsystem. */
  public static void runSimulationPeriodically() {
    for (VirtualSubsystem subsystem : virtualSubsystems) {
      subsystem.simulationPeriodic();
    }
  }

  /** Method to run on every thread update. */
  public abstract void periodic();

  /** Method to run on every simulated thread update. */
  public abstract void simulationPeriodic();
}
