package frc.robot.Romi;

import edu.wpi.first.wpilibj.RobotBase;
import frc.jonb.sysid.SysIdRobot;

public class RomiSysIdApp {
	public static void main(String... args) {
		RomiDriveSubsystem drive = new RomiDriveSubsystem();
		SysIdRobot robot = new SysIdRobot(drive);
		
		RobotBase.startRobot(() -> robot);
	}
}
