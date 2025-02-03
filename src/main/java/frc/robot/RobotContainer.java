// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.jonb.pathplanner.PPBridge;
import frc.jonb.subsystems.DiffDriveSubsystem;
import frc.jonb.sysid.SysIdDrivable;
import frc.robot.Robot.RobotType;
import frc.robot.Romi.RomiRobot;
import frc.robot.Xrp.XrpRobot;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.AutoDuration;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveDuration;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.TurnDuration;

/**
 * Build and configures the guts (subsystems) and brain (logic) of the robot.
 * User control is via dashboard choosers and a joystick.
 * <ul>
 * <li>Auto Mode Commands: The command executed each time Autonomous mode is
 * enabled.
 * <li>Test Mode Commands: The command executed each time Test mode is
 * enabled. To generate a SysId log, must run each of the four SysId commands to
 * completion.
 * </ul>
 */
public class RobotContainer {
	/**
	 * Creates an instance.
	 * 
	 * @param type
	 *            Robot type.
	 */
	public RobotContainer(RobotType type) {
		// build robot guts
		switch (type) {
			case ROMI:
				RomiRobot romiRobot = new RomiRobot();
				_sysidDrive = romiRobot.getSysidDrivetrain();
				_diffDrive = new DiffDriveSubsystem(
						romiRobot.getRawDrivetrain());
				break;
			case SWERVE:
				throw new IllegalStateException(
						"RobotType[" + type + "] is not yet implemented.");
			case XRP:
				XrpRobot xrpRobot = new XrpRobot();
				_sysidDrive = xrpRobot.getSysidDrivetrain();
				_diffDrive = new DiffDriveSubsystem(
						xrpRobot.getRawDrivetrain());
				break;
			default:
				throw new IllegalStateException(
						"RobotType[" + type + "] is unknown.");
		}

		// connect PathPlanner
		PPBridge.buildBridge(_diffDrive);

		// build UI
		_autoChooser = buildAutoChooser();
		_testChooser = buildTestChooser();
		_stick = new Joystick(0);
	}

	/**
	 * Gets the current command selected in the Auto Mode chooser.
	 *
	 * @return The command. None if null.
	 */
	public Command getAutonomousCommand() {
		return _autoChooser.getSelected();
	}

	/**
	 * Gets the current command selected in the Auto Mode chooser.
	 *
	 * @return The command. None if null.
	 */
	public Command getTeleopCommand() {
		return new ArcadeDrive(_diffDrive, _stick::getX, _stick::getY);
	}

	/**
	 * Gets the current command selected in the Test Mode chooser.
	 *
	 * @return The command. None if null.
	 */
	public Command getTestCommand() {
		return _testChooser.getSelected();
	}

	// personal

	/**
	 * Builds the SmartDashboard chooser for Auto mode. MUST first build
	 * PPBridge.
	 */
	protected SendableChooser<Command> buildAutoChooser() {
		SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.setDefaultOption("PathPlanner",
				new PathPlannerAuto("LoopAutoPath"));
		chooser.addOption("AutoDistance (0.5m)",
				new AutoDistance(_diffDrive, 0.5, 0.5));
		chooser.addOption("DriveDistance (+0.5m)",
				new DriveDistance(_diffDrive, 0.5, 0.5));
		chooser.addOption("DriveDistance (-0.5m)",
				new DriveDistance(_diffDrive, 0.5, -0.5));
		chooser.addOption("TurnAngle (+90deg)",
				new TurnAngle(_diffDrive, 0.5, 90.0));
		chooser.addOption("TurnAngle (-90deg)",
				new TurnAngle(_diffDrive, 0.5, -90.0));

		double targetSec = 4.0;
		double targetMps = 0.125;
		double targetFac = targetMps / _diffDrive.getDrive()
				.getWheelVelocityMax();

		chooser.addOption("AutoDuration (" + targetSec + "s)",
				new AutoDuration(_diffDrive, targetFac, targetSec));
		chooser.addOption("DriveDuration (" + targetSec + "s)",
				new DriveDuration(_diffDrive, targetFac, targetSec));
		chooser.addOption("TurnDuration (" + targetSec + "s)",
				new TurnDuration(_diffDrive, targetFac, targetSec));

		//// SmartDashboard.putData(chooser);
		SmartDashboard.putData("Auto Mode Commands", chooser);
		return chooser;
	}

	/**
	 * Builds the SmartDashboard chooser for Test mode.
	 */
	protected SendableChooser<Command> buildTestChooser() {
		SysIdRoutine sysidFactory = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						_sysidDrive::setVoltage,
						_sysidDrive::logEntry,
						_sysidDrive));

		SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.setDefaultOption("Quasistatic, Forward",
				sysidFactory.quasistatic(Direction.kForward));
		chooser.addOption("Quasistatic, Reverse",
				sysidFactory.quasistatic(Direction.kReverse));
		chooser.addOption("Dynamic, Forward",
				sysidFactory.dynamic(Direction.kForward));
		chooser.addOption("Dynamic, Reverse",
				sysidFactory.dynamic(Direction.kReverse));

		//// SmartDashboard.putData(chooser);
		SmartDashboard.putData("Test Mode Commands", chooser);
		return chooser;
	}

	private final DiffDriveSubsystem _diffDrive;
	private final SysIdDrivable _sysidDrive;
	private final SendableChooser<Command> _autoChooser;
	private final SendableChooser<Command> _testChooser;
	private final Joystick _stick;
}
