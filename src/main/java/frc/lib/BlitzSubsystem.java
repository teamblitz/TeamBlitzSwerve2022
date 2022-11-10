package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface BlitzSubsystem extends Subsystem, Sendable {
    /**
     * Initializes the telemetry for this subsystem.
     */
    default void initTelemetry() {};
}
