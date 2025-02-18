// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.romi;

import edu.wpi.first.wpilibj.romi.OnBoardIO;
import edu.wpi.first.wpilibj.romi.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.jonb.subsystems.DiffDrivable;
import frc.jonb.sysid.SysIdDrivable;

/**
 * A robot based on the Romi framework, with drivetraoin and peripheral
 * subsystem delegates.
 */
public class RomiRobot {
	/**
	 * Creates an instance.
	 */
	public RomiRobot() {
		_rawDrive = new RomiDriveSubsystem();
		_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
		configButtons();
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

	/**
	 * Configures onboard buttons.
	 * <p>
	 * NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the
	 * hardware "overlay" that is specified when launching the wpilib-ws server
	 * on
	 * the Romi raspberry pi. By default, the following are available (listed in
	 * order from inside of the board to outside):
	 * - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
	 * - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
	 * - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
	 * - PWM 2 (mapped to Arduino Pin 21)
	 * - PWM 3 (mapped to Arduino Pin 22)
	 * Your subsystem configuration should take the overlays into account
	 */
	protected void configButtons() {
		// Example of how to use the onboard IO
		new Trigger(
				_onboardIO::getButtonAPressed)
				.onTrue(new PrintCommand("Button A Pressed"))
				.onFalse(new PrintCommand("Button A Released"));
	}

	// personal

	private final RomiDriveSubsystem _rawDrive;
	private final OnBoardIO _onboardIO;
}
