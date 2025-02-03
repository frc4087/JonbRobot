// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Xrp;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.subsystems.DiffDrivable;
import frc.jonb.sysid.SysIdDrivable;

/**
 * A DriveSubsystem for an XRP robot.
 * <p>
 * Based on frc.robot.subsystems.RomiDriveSubsystem with mods based on
 * WPILIB generated XRP project.
 */
public class XrpDriveSubsystem extends SubsystemBase
		implements DiffDrivable, SysIdDrivable {
	private static final double WHEEL_GEAR_RATIO = (30.0 / 14.0) * (28.0 / 16.0)
			* (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
	private static final double WHEEL_MOTOR_CPR = 12.0;
	public static final double WHEEL_ENCODER_CPR = WHEEL_MOTOR_CPR
			* WHEEL_GEAR_RATIO; // 585.0

	public static final double WHEEL_TRACKWIDTH_M = 0.154;
	public static final double WHEEL_DIAMETER_M = 0.060;
	public static final double WHEEL_CIRCUMFERENCE_M = Math.PI
			* WHEEL_DIAMETER_M;
	public static final double WHEEL_RPS_MAX = 200.0 / 60.0; // from WPI docs
	public static final double WHEEL_MPS_MAX = WHEEL_RPS_MAX
			* WHEEL_CIRCUMFERENCE_M;

	public static final double LEFT_KS = 0.73087; // 0.8029;
	public static final double LEFT_KV = 10.334; // 10.076;
	public static final double LEFT_KA = 1.5703; // 2.0878;
	public static final double LEFT_KP = 2.9247; // 2.7555;
	public static final double LEFT_KI = 0.0000;
	public static final double LEFT_KD = 0.0000;

	public static final double RIGHT_KS = 0.79424; // 0.4714;
	public static final double RIGHT_KV = 7.5; // 9.8306; //9.7982;
	public static final double RIGHT_KA = 1.4868; // 2.5502;
	public static final double RIGHT_KP = 2.7573; // 3.8658;
	public static final double RIGHT_KI = 0.00000;
	public static final double RIGHT_KD = 0.00000;

	/** Creates a new Drivetrain. */
	public XrpDriveSubsystem() {
		_subsystems.add(this);

		_leftMotor = new XRPMotor(0);
		_leftEncoder = new Encoder(4, 5);
		_leftEncoder
				.setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
		_leftFeedforward = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV,
				LEFT_KA);
		_leftVelocityPid = new PIDController(LEFT_KP, LEFT_KI, LEFT_KD);

		_rightMotor = new XRPMotor(1);
		_rightMotor.setInverted(true);
		_rightEncoder = new Encoder(6, 7);
		_rightEncoder
				.setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
		_rightFeedforward = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV,
				RIGHT_KA);
		_rightVelocityPid = new PIDController(RIGHT_KP, RIGHT_KI, RIGHT_KD);

		resetEncoders();
	}

	// DiffDrivable

	@Override
	public double getTrackWidth() {
		return WHEEL_TRACKWIDTH_M;
	}

	@Override
	public double getWheelVelocityMax() {
		return WHEEL_MPS_MAX;
	}

	@Override
	public void setWheelVelocity(double leftMps, double rightMps) {
		double leftFfw = _leftFeedforward.calculate(leftMps);
		double rightFfw = _rightFeedforward.calculate(rightMps);

		double leftPid = _leftVelocityPid.calculate(getLeftVelocity(),
				leftMps);
		double rightPid = _rightVelocityPid.calculate(getRightVelocity(),
				rightMps);

		// System.out.printf(
		// "spd=%5.2f err=%5.2f pid=%5.2f ffw=%5.2f vlt=%6.2f\n",
		// leftMps, leftMps - getLeftVelocity(), leftPid, leftFfw,
		// leftPid + leftFfw);

		_leftMotor.setVoltage(leftFfw + leftPid);
		_rightMotor.setVoltage(rightFfw + rightPid);
	}

	@Override
	public void resetEncoders() {
		_leftEncoder.reset();
		_rightEncoder.reset();
	}

	@Override
	public double getLeftDistance() {
		return _leftEncoder.getDistance();
	}

	@Override
	public double getRightDistance() {
		return _rightEncoder.getDistance();
	}

	@Override
	public double getLeftVelocity() {
		return _leftEncoder.getRate();
	}

	@Override
	public double getRightVelocity() {
		return _rightEncoder.getRate();
	}

	@Override
	public void resetGyro() {
		_gyro.reset();
	}

	@Override
	public Rotation2d getRotationZ() {
		// XRP is normal
		return Rotation2d.fromDegrees(_gyro.getAngleZ());
	}

	@Override
	public Subsystem[] getSubsystems() {
		return _subsystems.toArray(Subsystem[]::new);
	}

	// SysIdDrivable

	@Override
	public void setVoltage(Voltage volts) {
		_leftMotor.setVoltage(volts);
		_rightMotor.setVoltage(volts);
	}

	@Override
	public void logEntry(SysIdRoutineLog log) {
		log.motor("drive-left")
				.voltage(_dummyVoltage.mut_replace(
						_leftMotor.get() * RobotController.getBatteryVoltage(),
						Units.Volts))
				.linearPosition(
						_dummyDistance.mut_replace(_leftEncoder.getDistance(),
								Units.Meters))
				.linearVelocity(
						_dummyVelocity.mut_replace(_leftEncoder.getRate(),
								Units.MetersPerSecond));
		log.motor("drive-right")
				.voltage(_dummyVoltage
						.mut_replace(_rightMotor.get() * RobotController
								.getBatteryVoltage(), Units.Volts))
				.linearPosition(
						_dummyDistance.mut_replace(_rightEncoder.getDistance(),
								Units.Meters))
				.linearVelocity(
						_dummyVelocity.mut_replace(_rightEncoder.getRate(),
								Units.MetersPerSecond));
	}

	// personal

	private final List<Subsystem> _subsystems = new ArrayList<>();
	private final XRPMotor _leftMotor;
	private final Encoder _leftEncoder;
	private final SimpleMotorFeedforward _leftFeedforward;
	private final PIDController _leftVelocityPid;

	private final XRPMotor _rightMotor;
	private final Encoder _rightEncoder;
	private final SimpleMotorFeedforward _rightFeedforward;
	private final PIDController _rightVelocityPid;

	private final XRPGyro _gyro = new XRPGyro();

	private final MutVoltage _dummyVoltage = Units.Volts.mutable(0);
	private final MutDistance _dummyDistance = Units.Meters.mutable(0);
	private final MutLinearVelocity _dummyVelocity = Units.MetersPerSecond
			.mutable(0);

}
