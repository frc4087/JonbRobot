// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Command that stops chassis movement and resets the drive pose to zero.
 */
public class StopAndReset extends Command {
	/**
	 * Creates an instance.
	 * @param drive The target drivetrain.
	 */
	public StopAndReset(CommandDrivable drive) {
		_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		_drive.stop();
		_drive.resetPose(Pose2d.kZero);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	// personal

	private final CommandDrivable _drive;
}
