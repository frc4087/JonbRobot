// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Command that turns CCW for a given duration at a given speed, with no
 * refinement (error correction, input profiling).
 */
public class RawTurnDuration extends Command {
	/**
	 * Creates an instance.
	 * @param drive The target drivetrain.
	 * @param speedFactor Speed factor relative to max [-1, +1].
	 * @param duration Distance (s, >=0).
	 */
	public RawTurnDuration(
			CommandDrivable drive, double speedFactor,
			double duration) {
		_drive = drive;
		addRequirements(drive);

		_speeds = new ChassisSpeeds(0.0, 0.0,
				drive.getMaxSpeeds().omegaRadiansPerSecond * speedFactor);
		_duration = duration * 1000;
	}

	@Override
	public void initialize() {
		_drive.stop();
		_startTime = System.currentTimeMillis();
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
		return (System.currentTimeMillis() - _startTime) >= _duration;
	}

	// personal
	
	private final CommandDrivable _drive;
	private final double _duration;
	private final ChassisSpeeds _speeds;
	private long _startTime;
}
