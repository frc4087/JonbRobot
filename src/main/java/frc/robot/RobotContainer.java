// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.jonb.pathplanner.PPBridge;
import frc.jonb.pathplanner.PPDrivable;
import frc.jonb.subsystems.DiffDriveSubsystem;
import frc.jonb.subsystems.HoloDriveSubsystem;
import frc.jonb.sysid.SysIdDrivable;
import frc.robot.Robot.RobotType;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RawDriveDistance;
import frc.robot.commands.RawDriveDuration;
import frc.robot.commands.RawTurnAngle;
import frc.robot.commands.RawTurnDuration;
import frc.robot.commands.ZeroPose;
import frc.robot.romi.RomiRobot;
import frc.robot.swerve.SwerveRobot;
import frc.robot.xrp.XrpRobot;

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
	 *             Robot type.
	 */
	public RobotContainer(RobotType type) {
		// build robot guts
		switch (type) {
			case ROMI:
				RomiRobot romiRobot = new RomiRobot();
				_sysidDrive = romiRobot.getSysidDrivetrain();
				_ppDrive = new DiffDriveSubsystem(
						romiRobot.getRawDrivetrain());
				break;
			case SWERVE:
				SwerveRobot swerveRobot = new SwerveRobot();
				_sysidDrive = swerveRobot.getSysidDrivetrain();
				_ppDrive = new HoloDriveSubsystem(
						swerveRobot.getRawDrivetrain());
				break;
			case XRP:
				XrpRobot xrpRobot = new XrpRobot();
				_sysidDrive = xrpRobot.getSysidDrivetrain();
				_ppDrive = new DiffDriveSubsystem(
						xrpRobot.getRawDrivetrain());
				break;
			default:
				throw new IllegalStateException(
						"RobotType[" + type + "] is unknown.");
		}

		// connect PathPlanner
		PPBridge.buildBridge(_ppDrive);

		// build UI
		_autoChooser = buildAutoChooser();
		_testChooser = buildTestChooser();
		_xbox = new CommandXboxController(0);

		/////////////////
		// CommandXboxController xbox = new CommandXboxController(0);
		// XboxController xboxHid = xbox.getHID();

		// xbox.a().onTrue(new InstantCommand(
		// 		() -> System.out.println("RobotContainer: Button A")));
		// xbox.povUp().onTrue(new InstantCommand(
		// 		() -> System.out.println("RobotContainer: POV Up")));
		// new Trigger(xboxHid.povRight(CommandScheduler.getInstance().getDefaultButtonLoop()))
		// 		.onTrue(new InstantCommand(
		// 				() -> System.out.println("RobotContainer: POV Right")));
		// new Trigger(xboxHid.pov(270, CommandScheduler.getInstance().getDefaultButtonLoop()))
		// 		.onTrue(new InstantCommand(
		//				() -> System.out.println("RobotContainer: POV 270")));
		/////////////////
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
		return new ArcadeDrive(_ppDrive, () -> -_xbox.getRightY(),
				() -> -_xbox.getRightX());
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

		chooser.addOption("PathPlanner",
				new PathPlannerAuto("LoopAutoPath"));
		chooser.addOption("DriveToPose (+0.5m, +90deg)",
				new ZeroPose(_ppDrive)
						.andThen(new DriveToPose(_ppDrive, new Pose2d(+0.5, 0.0, Rotation2d.fromDegrees(0.0)), 0.5)));
		chooser.addOption("DriveDistance (+0.5m)",
				new RawDriveDistance(_ppDrive, +0.5, 0.5));
		chooser.addOption("DriveDistance (-0.5m)",
				new RawDriveDistance(_ppDrive, -0.5, 0.5));
		chooser.addOption("TurnAngle (+90deg)",
				new RawTurnAngle(_ppDrive, 0.5, 90.0));
		chooser.addOption("TurnAngle (-90deg)",
				new RawTurnAngle(_ppDrive, -0.5, 90.0));

		double targetSec = 2.0;

		double targetMps = 0.25;
		double targetDriveFac = targetMps
				/ _ppDrive.getMaxSpeeds().vxMetersPerSecond;

		double targetDps = 90.0;
		double targetTurnFac = targetDps / Math.toDegrees(_ppDrive
				.getMaxSpeeds().omegaRadiansPerSecond);

		chooser.addOption(
				"DriveDuration (" + targetSec + "s, " + targetMps * targetSec
						+ "m)",
				new RawDriveDuration(_ppDrive, targetDriveFac, targetSec));
		chooser.addOption(
				"TurnDuration (" + targetSec + "s, " + targetDps * targetSec
						+ "deg)",
				new RawTurnDuration(_ppDrive, targetTurnFac, targetSec));

		SmartDashboard.putData("Auto Mode Commands", chooser);
		return chooser;
	}

	/**
	 * Builds the SmartDashboard chooser for Test mode.
	 */
	protected SendableChooser<Command> buildTestChooser() {
		SysIdRoutine sysidFactory = new SysIdRoutine(new SysIdRoutine.Config(
				Volts.of(1).per(Second), Volts.of(7), Seconds.of(7)),
				new SysIdRoutine.Mechanism(
						_sysidDrive::setVoltage,
						_sysidDrive::logEntry,
						_sysidDrive));

		SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption("Quasistatic, Forward",
				sysidFactory.quasistatic(Direction.kForward));
		chooser.addOption("Quasistatic, Reverse",
				sysidFactory.quasistatic(Direction.kReverse));
		chooser.addOption("Dynamic, Forward",
				sysidFactory.dynamic(Direction.kForward));
		chooser.addOption("Dynamic, Reverse",
				sysidFactory.dynamic(Direction.kReverse));

		SmartDashboard.putData("Test Mode Commands", chooser);
		return chooser;
	}

	private final PPDrivable _ppDrive;
	private final SysIdDrivable _sysidDrive;
	private final SendableChooser<Command> _autoChooser;
	private final SendableChooser<Command> _testChooser;
	private final CommandXboxController _xbox;
}
