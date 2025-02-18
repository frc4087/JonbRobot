// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.subsystems.HoloDrivable;
import frc.jonb.sysid.SysIdDrivable;

/**
 * A DriveSubsystem for an XRP robot.
 * <p>
 * Based on frc.robot.subsystems.RomiDriveSubsystem with mods based on
 * WPILIB generated XRP project.
 */
public class SwerveDriveSubsystem extends SubsystemBase
		implements HoloDrivable, SysIdDrivable {
	// private static final double WHEEL_GEAR_RATIO = (30.0 / 14.0) * (28.0 / 16.0)
	// 		* (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
	// private static final double WHEEL_MOTOR_CPR = 12.0;
	// public static final double WHEEL_ENCODER_CPR = WHEEL_MOTOR_CPR
	// 		* WHEEL_GEAR_RATIO; // 585.0

	// public static final double WHEEL_TRACKWIDTH_M = 0.154;
	// public static final double WHEEL_DIAMETER_M = 0.060;
	// public static final double WHEEL_CIRCUMFERENCE_M = Math.PI
	// 		* WHEEL_DIAMETER_M;
	// public static final double WHEEL_RPS_MAX = 200.0 / 60.0; // from WPI docs
	// public static final double WHEEL_MPS_MAX = WHEEL_RPS_MAX
	// 		* WHEEL_CIRCUMFERENCE_M;

	// public static final double LEFT_KS = 1.0494;
	// public static final double LEFT_KV = 9.9313;
	// public static final double LEFT_KA = 2.6492;
	// public static final double LEFT_KP = 9.3688; // V/M/S
	// public static final double LEFT_KI = 0.002;
	// public static final double LEFT_KD = 0.0000;

	// public static final double RIGHT_KS = 0.79424;
	// public static final double RIGHT_KV = 7.5;
	// public static final double RIGHT_KA = 1.4868;
	// public static final double RIGHT_KP = 2.7573; // V/M/S
	// public static final double RIGHT_KI = 0.002;
	// public static final double RIGHT_KD = 0.00000;

	/** Creates a new Drivetrain. */
	public SwerveDriveSubsystem() {
		_subsystems.add(this);

		// _leftMotor = new XRPMotor(0);
		// _leftEncoder = new Encoder(4, 5);
		// _leftEncoder
		// 		.setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
		// _leftFeedforward = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV,
		// 		LEFT_KA);
		// _leftVelocityPid = new PIDController(LEFT_KP, LEFT_KI, LEFT_KD);

		// _rightMotor = new XRPMotor(1);
		// _rightMotor.setInverted(true);
		// _rightEncoder = new Encoder(6, 7);
		// _rightEncoder
		// 		.setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
		// _rightFeedforward = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV,
		// 		RIGHT_KA);
		// _rightVelocityPid = new PIDController(RIGHT_KP, RIGHT_KI, RIGHT_KD);

		// resetEncoders();
	}

	// // DiffDrivable

	// @Override
	// public double getTrackWidth() {
	// 	return WHEEL_TRACKWIDTH_M;
	// }

	// @Override
	// public double getWheelVelocityMax() {
	// 	return WHEEL_MPS_MAX;
	// }

	// @Override
	// public void setWheelVelocity(double leftMps, double rightMps) {
	// 	Voltage leftFfw = _leftFeedforward
	// 			.calculate(Units.MetersPerSecond.of(leftMps));
	// 	Voltage rightFfw = _rightFeedforward
	// 			.calculate(Units.MetersPerSecond.of(rightMps));

	// 	double leftPidVolts = _leftVelocityPid.calculate(getLeftVelocity(),
	// 			leftMps);
	// 	double rightPidVolts = _rightVelocityPid.calculate(getRightVelocity(),
	// 			rightMps);

	// 	double leftVolts = leftFfw.in(Units.Volts) + leftPidVolts;
	// 	double rightVolts = rightFfw.in(Units.Volts) + rightPidVolts;

	// 	////System.out.printf("XrpDriveSubsystem: l= %6.2f %6.2f, r= %6.2f %6.2f; vel= %6.2f %6.2f\n", 
	// 	////leftFfw.in(Units.Volts), leftPidVolts, rightFfw.in(Units.Volts), rightPidVolts,
	// 	////getLeftVelocity(), getRightVelocity());

	// 	_leftMotor.setVoltage(leftVolts);
	// 	_rightMotor.setVoltage(rightVolts);
	// }

	// @Override
	// public void resetControllers() {
	// 	_leftVelocityPid.reset();
	// 	_rightVelocityPid.reset();
	// }

	// @Override
	// public void resetEncoders() {
	// 	_leftEncoder.reset();
	// 	_rightEncoder.reset();
	// }

	// @Override
	// public double getLeftDistance() {
	// 	return _leftEncoder.getDistance();
	// }

	// @Override
	// public double getRightDistance() {
	// 	return _rightEncoder.getDistance();
	// }

	// @Override
	// public double getLeftVelocity() {
	// 	return _leftEncoder.getRate();
	// }

	// @Override
	// public double getRightVelocity() {
	// 	return _rightEncoder.getRate();
	// }

	// @Override
	// public void resetGyro() {
	// 	_gyro.reset();
	// }

	// @Override
	// public Rotation2d getRotationZ() {
	// 	// XRP is normal
	// 	return Rotation2d.fromDegrees(_gyro.getAngleZ());
	// }

	@Override
	public List<Subsystem> getSubsystems() {
		return Collections.unmodifiableList(_subsystems);
	}

	// SysIdDrivable

	@Override
	public void setVoltage(Voltage volts) {
		// _leftMotor.setVoltage(volts);
		// _rightMotor.setVoltage(volts);
	}

	@Override
	public void logEntry(SysIdRoutineLog log) {
		// log.motor("drive-left")
		// 		.voltage(_dummyVoltage.mut_replace(
		// 				_leftMotor.get() * RobotController.getBatteryVoltage(),
		// 				Units.Volts))
		// 		.linearPosition(
		// 				_dummyDistance.mut_replace(_leftEncoder.getDistance(),
		// 						Units.Meters))
		// 		.linearVelocity(
		// 				_dummyVelocity.mut_replace(_leftEncoder.getRate(),
		// 						Units.MetersPerSecond));
		// log.motor("drive-right")
		// 		.voltage(_dummyVoltage
		// 				.mut_replace(_rightMotor.get() * RobotController
		// 						.getBatteryVoltage(), Units.Volts))
		// 		.linearPosition(
		// 				_dummyDistance.mut_replace(_rightEncoder.getDistance(),
		// 						Units.Meters))
		// 		.linearVelocity(
		// 				_dummyVelocity.mut_replace(_rightEncoder.getRate(),
		// 						Units.MetersPerSecond));
	}

	// personal

	private final List<Subsystem> _subsystems = new ArrayList<>();

	// private final XRPMotor _leftMotor;
	// private final Encoder _leftEncoder;
	// private final SimpleMotorFeedforward _leftFeedforward;
	// private final PIDController _leftVelocityPid;

	// private final XRPMotor _rightMotor;
	// private final Encoder _rightEncoder;
	// private final SimpleMotorFeedforward _rightFeedforward;
	// private final PIDController _rightVelocityPid;

	// private final XRPGyro _gyro = new XRPGyro();

	// private final MutVoltage _dummyVoltage = Units.Volts.mutable(0);
	// private final MutDistance _dummyDistance = Units.Meters.mutable(0);
	// private final MutLinearVelocity _dummyVelocity = Units.MetersPerSecond
	// 		.mutable(0);

}
