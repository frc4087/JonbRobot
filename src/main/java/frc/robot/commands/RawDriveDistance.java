// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Command that drives forward a given distance at a given speed, with no
 * refinement (error correction, input profiling).
 */
public class RawDriveDistance extends Command {
	/**
	 * Creates an instance.
	 * @param drive The target drivetrain.
	 * @param speedFactor Speed factor relative to max [-1, +1].
	 * @param distance Travel distance (m, >=0) relative to current robot pose.
	 *            Sign is ignored.
	 */
	public RawDriveDistance(CommandDrivable drive, double speedFactor,
			double distance) {
		_drive = drive;
		addRequirements(drive);

		_speeds = new ChassisSpeeds(
				drive.getMaxSpeeds().vxMetersPerSecond * speedFactor, 0.0, 0.0);
		_distance = Math.abs(distance);
	}

	@Override
	public void initialize() {
		_drive.stop();
		_drive.resetPose(Pose2d.kZero); // X is forward
	}

	@Override
	public void execute() {
		_drive.setDriveSpeeds(_speeds);
	}

	@Override
	public void end(boolean interrupted) {
		_drive.stop();
	}

	@Override
	public boolean isFinished() {
		double distance = _drive.getPose().getX();
		double error = _distance - (Math.signum(_speeds.vxMetersPerSecond) * distance);
		// System.out.println("DriveDistance: " +
		// "goal=" + _distance + " now=" + distance + " err=" + error);
		return error <= 0; // at or beyond distance
	}

	// personal

	private final CommandDrivable _drive;
	private final double _distance;
	private final ChassisSpeeds _speeds;

}
