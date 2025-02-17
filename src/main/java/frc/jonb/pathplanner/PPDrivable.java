package frc.jonb.pathplanner;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Interface for a drivetrain (differential or holonomic) that can support
 * PathPlanner.
 */
public interface PPDrivable extends CommandDrivable {
	/**
	 * Gets the actual robot relative velocities (not neccesarily
	 * that set by setDriveSpeeds()).
	 * @return The velocities.
	 */
	ChassisSpeeds getTrueSpeeds();
}