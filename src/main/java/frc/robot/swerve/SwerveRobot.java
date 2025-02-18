// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import frc.jonb.subsystems.HoloDrivable;
import frc.jonb.sysid.SysIdDrivable;

/**
 * A robot based on the Swerve framework, with drivetraoin and peripheral
 * subsystem delegates.
 */
public class SwerveRobot {
	/**
	 * Creates an instance.
	 */
	public SwerveRobot() {
		_rawDrive = new SwerveDriveSubsystem();
	}

	/**
	 * Gets the "raw" robot drivetrain, used for normal robot operation.
	 * 
	 * @return The object.
	 */
	public HoloDrivable getRawDrivetrain() {
		return _rawDrive;
	}

	/**
	 * Gets the "sysid" robot drivetrain, used for robot characterization.
	 * 
	 * @return The object.
	 */
	public SysIdDrivable getSysidDrivetrain() {
		return _rawDrive;
	}

	// personal

	private final SwerveDriveSubsystem _rawDrive;
}
