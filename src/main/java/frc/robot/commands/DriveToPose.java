// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.CommandDrivable;

/**
 * Commands the robot to drive from its current pose to a new pose at a given
 * speed. The nature of the command does not lend itself to chaining, as in a
 * path. Use PathPlanner instead. As such, zeros the speed at the beginning and
 * end of the command, but does not reset pose. As needed, use separate commands
 * before or after this call to reset the pose (e.g. ZeroPose).
 * <p>
 * Makes generous assumptions about error tolerance and PID tuning.
 * <p>
 * TODO: Delegate profile and controller building to the drive since it best
 * knows its needs and constraints.
 */
public class DriveToPose extends Command {
	/**
	 * Creates an instance.
	 * 
	 * @param drivetrain  The target drivetrain.
	 * @param speedFactor Speed factor relative to max [0, +1]. Sign is ignored.
	 * @param pose        New pose.
	 */
	public DriveToPose(CommandDrivable drive, Pose2d pose,
			double speedFactor) {
		_drive = drive;
		addRequirements(drive);

		_poseEnd = pose;
		_speeds = _drive.getMaxSpeeds().times(speedFactor);

		// build controller
		Pose2d tolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3.0));
		if (_drive.isHolonomic()) {
			PIDController pidX = new PIDController(1.0, 0.0, 0.0);
			PIDController pidY = new PIDController(1.0, 0.0, 0.0);
			ProfiledPIDController pidZ = new ProfiledPIDController(2.0, 0.0,
					0.0, new TrapezoidProfile.Constraints(3.0, 3.0));
			pidZ.enableContinuousInput(-Math.PI, +Math.PI);

			_holoPid = new HolonomicDriveController(pidX, pidY, pidZ);
			_holoPid.setTolerance(tolerance);
		} else {
			_diffPid = new LTVUnicycleController(
					// VecBuilder.fill(0.01, 0.01, Math.toRadians(5.0)),
					// VecBuilder.fill(_drive.getMaxSpeeds().vxMetersPerSecond,
					// _drive.getMaxSpeeds().omegaRadiansPerSecond),
					VecBuilder.fill(0.05, 0.05, 0.05),
					VecBuilder.fill(0.25, 0.25),
					0.02, _speeds.vxMetersPerSecond);
			_diffPid.setTolerance(tolerance);
		}

		// System.out.printf("DriveToPose: %6.2f %6.2f %6.1f\n", _poseEnd.getX(),
		// _poseEnd.getY(),
		// _poseEnd.getRotation().getDegrees());
	}

	@Override
	public void initialize() {
		_drive.stop();
	}

	@Override
	public void execute() {
		Pose2d poseNow = _drive.getPose();
		ChassisSpeeds speeds;
		if (_drive.isHolonomic()) {
			speeds = _holoPid.calculate(poseNow, _poseEnd,
					_speeds.vxMetersPerSecond, _poseEnd.getRotation());
		} else {
			speeds = _diffPid.calculate(poseNow, _poseEnd, 0.0, 0.0);
		}
		_drive.setDriveSpeeds(speeds);

		// report error
		// Pose2d err = _poseEnd.relativeTo(poseNow);
		// System.out.printf(" err= %6.2f %6.2f %6.1f; spd= %6.2f %6.2f %6.1f\n",
		// err.getX(), err.getY(),
		// err.getRotation().getDegrees(), speeds.vxMetersPerSecond,
		// speeds.vyMetersPerSecond, Math.toDegrees(speeds.omegaRadiansPerSecond),
		// poseNow.getX(), poseNow.getY(),
		// poseNow.getRotation().getDegrees());
	}

	@Override
	public void end(boolean interrupted) {
		_drive.stop();
	}

	@Override
	public boolean isFinished() {
		if (_drive.isHolonomic()) {
			return _holoPid.atReference();
		} else {
			return _diffPid.atReference();
		}
	}

	// personal

	private final CommandDrivable _drive;
	private final Pose2d _poseEnd;
	private final ChassisSpeeds _speeds;

	private LTVUnicycleController _diffPid;
	private HolonomicDriveController _holoPid;
}
