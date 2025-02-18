// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Command supporting "arcade" style (forward and turn) drive control, such
 * as from a hand controller. Runs until explicitely terminated.
 */
public class ArcadeDrive extends Command {
	private final CommandDrivable _drivetrain;
	private final Supplier<Double> _forwardSource;
	private final Supplier<Double> _turnSource;

	/**
	 * Creates an instance.
	 * 
	 * @param drivetrain
	 *                      The target drivetrain.
	 * @param forwardSource
	 *                      Supplier of forward speed factor relative to max [-1,
	 *                      +1].
	 * @param turnSource
	 *                      Supplier of CCW turn speed factor relative to max [-1,
	 *                      +1].
	 */
	public ArcadeDrive(
			CommandDrivable drivetrain,
			Supplier<Double> forwardSource,
			Supplier<Double> turnSource) {
		_drivetrain = drivetrain;
		_forwardSource = forwardSource;
		_turnSource = turnSource;
		addRequirements(drivetrain);
	}

	// Command

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double fwdMps = _drivetrain.getMaxSpeeds().vxMetersPerSecond * _forwardSource.get();
		double ccwRps = _drivetrain.getMaxSpeeds().omegaRadiansPerSecond * _turnSource.get();
		
		_drivetrain.setDriveSpeeds(new ChassisSpeeds(fwdMps, 0.0, ccwRps));
	}

	@Override
	public void end(boolean interrupted) {
		// nothing to do
	}

	@Override
	public boolean isFinished() {
		return false; // run forever
	}
}
