// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/*
 * Creates a new TurnTime command. This command will turn your robot for a
 * desired rotational speed and time.
 */
public class TurnDuration extends Command {
	private final double _duration;
	private final double _speed;
	private final DiffDriveSubsystem _drive;
	private long m_startTime;

	/**
	 * Command that turns CCW for a given duration at a given speed.
	 */
	public TurnDuration(DiffDriveSubsystem drive, double speedFactor,
			double duration) {
		_speed = speedFactor;
		_duration = duration * 1000;
		_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_startTime = System.currentTimeMillis();
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public void execute() {
		_drive.arcadeDrive(0, _speed);
	}

	@Override
	public void end(boolean interrupted) {
		// stop drive
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() - m_startTime) >= _duration;
	}
}
