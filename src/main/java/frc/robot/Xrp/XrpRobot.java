// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.xrp;

import frc.jonb.subsystems.DiffDrivable;
import frc.jonb.sysid.SysIdDrivable;

/**
 * A robot based on the XRP framework, with drivetraoin and peripheral
 * subsystem delegates.
 */
public class XrpRobot {
	/**
	 * Creates an instance.
	 */
	public XrpRobot() {
		_rawDrive = new XrpDriveSubsystem();
	}

	/**
	 * Gets the "raw" robot drivetrain, used for normal robot operation.
	 * 
	 * @return The object.
	 */
	public DiffDrivable getRawDrivetrain() {
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

	private final XrpDriveSubsystem _rawDrive;
}
