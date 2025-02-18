package frc.jonb.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.pathplanner.PPDrivable;

/**
 * A holonomic drive Subsystem that wraps a target HoloDrivable to provide
 * high level drive modes and PathPlanner support.
 */
public class HoloDriveSubsystem extends SubsystemBase implements PPDrivable {
	/**
	 * Creates an instance.
	 * 
	 * @param drive
	 *              The differential drivetrain target.
	 */
	public HoloDriveSubsystem(HoloDrivable drive) {
		ErrorMessages.requireNonNullParam(
				drive, "drive", "HoloDrivable");
		_drive = drive;

		_subsystems.add(this);
		_subsystems.addAll(_drive.getSubsystems());

		// _kinematics = new DifferentialDriveKinematics(
		// 		_drive.getTrackWidth());
		// _odometry = new DifferentialDriveOdometry(
		// 		new Rotation2d(), 0.0, 0.0);

		// double speedMaxFwd = _kinematics
		// 		.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
		// 				_drive.getWheelVelocityMax(),
		// 				_drive.getWheelVelocityMax())).vxMetersPerSecond;
		// double speedMaxCcw = _kinematics
		// 		.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
		// 				-_drive.getWheelVelocityMax(),
		// 				+_drive.getWheelVelocityMax())).omegaRadiansPerSecond;
		// _speedsMax = new ChassisSpeeds(speedMaxFwd, 0.0, speedMaxCcw);
	}

	/**
	 * Gets the target drivetrain.
	 * 
	 * @return Shared exposed drivetrain.
	 */
	public HoloDrivable getDrive() {
		return _drive;
	}

	// /**
	//  * Sets robot chassis speed factors.
	//  * 
	//  * @param forwardFactor
	//  *                      Forward speed factor [-1,+1].
	//  * @param rotateSpeed
	//  *                      Rotation CCW speed factor [-1,+1].
	//  */
	// public void arcadeDrive(double forwardFactor, double ccwFactor) {
	// 	WheelSpeeds wheelSpeedFactors = DifferentialDrive.arcadeDriveIK(
	// 			forwardFactor, ccwFactor, true);
	// 	_drive.setWheelVelocity(
	// 			wheelSpeedFactors.left * _drive.getWheelVelocityMax(),
	// 			wheelSpeedFactors.right * _drive.getWheelVelocityMax());
	// }

	// /**
	//  * Sets robot wheel speed factors.
	//  * 
	//  * @param leftFactor
	//  *                    Left wheel forward speed factor [-1,+1].
	//  * @param rightFactor
	//  *                    Right wheel forward speed factor [-1,+1].
	//  */
	// public void tankDrive(double leftFactor, double rightFactor) {
	// 	_drive.setWheelVelocity(
	// 			leftFactor * _drive.getWheelVelocityMax(),
	// 			rightFactor * _drive.getWheelVelocityMax());
	// }

	// PPDrivable

	@Override
	public ChassisSpeeds getTrueSpeeds() {
		// DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
		// 		_drive.getLeftVelocity(), _drive.getRightVelocity());
		// return _kinematics.toChassisSpeeds(wheelSpeeds);
		return null;
	}

	// CommandDrivable

	@Override
	public void resetPose(Pose2d pose) {
		// _drive.resetGyro();
		// _drive.resetEncoders();

		// _odometry.resetPosition(_drive.getRotationZ(),
		// 		_drive.getLeftDistance(),
		// 		_drive.getRightDistance(), pose);
	}

	@Override
	public Pose2d getPose() {
		// return _odometry.getPoseMeters();
		return null;
	}

	@Override
	public void setDriveSpeeds(ChassisSpeeds speeds) {
		// DifferentialDriveWheelSpeeds wheelSpeeds = _kinematics
		// 		.toWheelSpeeds(speeds);
		// _drive.setWheelVelocity(wheelSpeeds.leftMetersPerSecond,
		// 		wheelSpeeds.rightMetersPerSecond);
	}

	@Override
	public ChassisSpeeds getMaxSpeeds() {
		//return _speedsMax;
		return null;
	}

	@Override
	public void stop() {
		// _drive.resetControllers();

		// // reset encoder velocity but maintain pose
		// Pose2d poseNow = getPose();
		// resetPose(poseNow);

		// // last, after all other resets
		// _drive.setWheelVelocity(0.0, 0.0);
	}

	@Override
	public boolean isHolonomic() {
		return false;
	}

	@Override
	public List<Subsystem> getSubsystems() {
		return Collections.unmodifiableList(_subsystems);
	}

	// Subsystem

	@Override
	public void periodic() {
		super.periodic();
		_drive.periodic();

		// _odometry.update(_drive.getRotationZ(), _drive.getLeftDistance(),
		// 		_drive.getRightDistance());
	}

	// personal

	private final HoloDrivable _drive;
	private final List<Subsystem> _subsystems = new ArrayList<>();
	//private final ChassisSpeeds _speedsMax;

	// private final DifferentialDriveKinematics _kinematics;
	// private final DifferentialDriveOdometry _odometry;
}
