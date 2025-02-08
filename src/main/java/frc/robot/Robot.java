// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
	private final RobotContainer m_robotContainer;
	private final RobotTelemetry m_robotTelemetry;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any
	 * initialization code.
	 */
	public Robot() {
		RobotType type = RobotType.XRP;
		m_robotContainer = new RobotContainer(type);
		m_robotTelemetry = new RobotTelemetry(type);
	}

	@Override
	public void robotInit() {
		m_robotTelemetry.telemetryInit();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for
	 * items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before
	 * LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from
		// the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		m_robotTelemetry.telemetryPeriodic();
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		Command command = m_robotContainer.getAutonomousCommand();
		if (command != null) {
			command.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();

		Command command = m_robotContainer.getTeleopCommand();
		if (command != null) {
			command.schedule();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();

		Command command = m_robotContainer.getTestCommand();
		if (command != null) {
			command.schedule();
		}
	}

	@Override
	public void testPeriodic() {}

	// class

	/**
	 * Specifies the specific technology base used for the robot.
	 */
	public static enum RobotType {
		ROMI, XRP, SWERVE;
	}
}
