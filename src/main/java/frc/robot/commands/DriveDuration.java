// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that drives forward for a given duration at a given speed.
 */
public class DriveDuration extends Command {
	private final double _duration;
	private final double _speed;
	private final DiffDriveSubsystem _drive;
	private long _startTime;

	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param duration
	 *            Distance (s, >=0).
	 */
	public DriveDuration(DiffDriveSubsystem drive, double speedFactor, double duration) {
		_speed = speedFactor;
		_duration = duration * 1000;
		_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		_startTime = System.currentTimeMillis();
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public void execute() {
		_drive.arcadeDrive(_speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		// stop drive
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() - _startTime) >= _duration;
	}
}
