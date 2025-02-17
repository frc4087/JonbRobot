// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.math.ContinuousRotation2d;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Command that turns CCW for a given angle at a given speed, with no
 * refinement (error correction, input profiling).
 */
public class RawTurnAngle extends Command {
	/**
	 * Creates an instance.
	 * @param drive The target drivetrain.
	 * @param speedFactor Speed factor relative to max [-1, +1].
	 * @param angle Angle (deg, >=0). Sign is ignored.
	 */
	public RawTurnAngle(CommandDrivable drive, double speedFactor,
			double angle) {
		_drive = drive;
		addRequirements(drive);

		_speeds = new ChassisSpeeds(0.0, 0.0,
				drive.getMaxSpeeds().omegaRadiansPerSecond * speedFactor);
		_angle = Math.abs(angle);

		_continuous = new ContinuousRotation2d(
				() -> _drive.getPose().getRotation());
	}

	@Override
	public void initialize() {
		_drive.stop();
		_drive.resetPose(Pose2d.kZero);
		_continuous.reset();
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
		double angle = _continuous.get().getDegrees();
		double error = _angle - (Math.signum(_speeds.omegaRadiansPerSecond) * angle);
		// System.out.println("TurnAngle: " +
		// "goal=" + _angle + " now=" + angle + " err=" + error);
		return error <= 0.0;
	}

	// personal

	private final CommandDrivable _drive;
	private final double _angle;
	private final ChassisSpeeds _speeds;

	private ContinuousRotation2d _continuous;
}
