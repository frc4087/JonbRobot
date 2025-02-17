package frc.robot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot.RobotType;

/**
 * Provides telemtry to the robot dashboard via network table entries.
 * <ul>
 * <li>WiFiMonitor: Network WIFI status.
 * </ul>
 */
public class RobotTelemetry {
	/**
	 * Creates an instance.
	 * 
	 * @param type
	 *            Robot type.
	 */
	public RobotTelemetry(RobotType type) {
		switch (type) {
			case ROMI:
				_robotIp = "10.0.0.2";
				break;
			case SWERVE:
				////System.out.println("RobotTelemetry: fix SWERVE");
				throw new IllegalStateException(
						"RobotType[" + type + "] is not yet implemented.");
			case XRP:
				_robotIp = "192.168.42.1";
				break;
			default:
				throw new IllegalStateException(
						"RobotType[" + type + "] is unknown.");
		}
	}

	public void telemetryInit() {
		var table = NetworkTableInstance.getDefault().getTable("WiFiMonitor");
		_pingStatus = table.getEntry("PingTime");
		_signalStatus = table.getEntry("SignalStrength");
		_netStatus = new WindowsNetStatus(SAMPLE_PERIOD_MS,
				NETWORK_ADAPTER_INDEX, _robotIp);
	}

	public void telemetryPeriodic() {
		_pingStatus.setDouble(_netStatus.getPingTime());
		_signalStatus.setDouble(_netStatus.getSignalStrength());
	}

	// personal

	private final String _robotIp;
	private WindowsNetStatus _netStatus;
	private NetworkTableEntry _pingStatus;
	private NetworkTableEntry _signalStatus;

	// class

	public static final int SAMPLE_PERIOD_MS = 1000; // 1 sample/sec
	public static final int NETWORK_ADAPTER_INDEX = 0; // first adapter
	////public static final int NETWORK_ADAPTER_INDEX = 1; // second adapter

	/**
	 * Object used for accessing robot wifi status. Executes long system calls
	 * on a separate thread so as not to interfere with robot periodic
	 * functions.
	 * <p>
	 * Note that, on the host system, app access to location is required to
	 * query for full network status. On Windows, see Settings->Privacy &
	 * Security->Location.
	 */
	public static class WindowsNetStatus {
		/**
		 * Creates an instance.
		 * 
		 * @param periodMs
		 *            Update thread sample period (>0 ms).
		 * @param adapterI
		 *            Index of the network adapter used for the robot connection
		 *            (>=0).
		 * @param robotIp
		 *            IP address of the robot wifi connection.
		 */
		public WindowsNetStatus(long periodMs, int adapterI, String robotIp) {
			String adapterName = getAdapterName(adapterI);
			if (adapterName == null) {
				System.out.println("WindowsNetStatus: Network adapter ["
						+ adapterI + "] not found.\n"
						+ adapterName);
				return; // do nothing
			}

			// run forever on separate thread
			CompletableFuture.runAsync(() -> {
				while (true) {
					_pingMs = getPingTime(robotIp);
					_signalPct = getSignalStrength(adapterName);
					try {
						Thread.sleep(periodMs);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			});
		}

		/**
		 * Gets the most recent ping time update of the connectrion.
		 * 
		 * @return Ping time (ms). <0 if no connection.
		 */
		public double getPingTime() {
			return _pingMs;
		}

		/**
		 * Gets the the most recent relative strength update of the connection.
		 * 
		 * @return Signal strength (%). <0 if no connnection.
		 */
		public double getSignalStrength() {
			return _signalPct;
		}

		// personal

		private double _pingMs = -1;
		private double _signalPct = -1;

		// class

		/**
		 * Gets the ping time for a given internet connection.
		 * 
		 * @param ipAddress
		 *            Connection IP address.
		 * @return Ping time (ms). <0 if no connection.
		 */
		public static double getPingTime(String ipAddress) {
			try {
				Process process = Runtime.getRuntime()
						.exec("ping -n 1 " + ipAddress);
				BufferedReader reader = new BufferedReader(
						new InputStreamReader(process.getInputStream()));

				// scan lines until "time=" found
				String line;
				while ((line = reader.readLine()) != null) {
					if (line.contains("time=")) {
						String[] parts = line.split("time=");
						return Double.parseDouble(parts[1].split("ms")[0]);
					}
				}
			} catch (Exception ex) {
				System.out.println("getPingTime(" + ipAddress + ") failed.\n"
						+ ex);
			}
			return -1;
		}

		/**
		 * Gets the signal strength of the connection for a given network
		 * adapter system name.
		 * 
		 * @param adapterName
		 *            Adapter system name.
		 * @return Signal strength (%). <0 if no connnection.
		 */
		public static int getSignalStrength(String adapterName) {
			try {
				Process process = Runtime.getRuntime()
						.exec("netsh wlan show interfaces");
				BufferedReader reader = new BufferedReader(
						new InputStreamReader(process.getInputStream()));

				// scan lines until adapter found, then scan lines until
				// "Signal" found
				String line;
				boolean isFound = false;
				while ((line = reader.readLine()) != null) {
					if (!isFound) {
						if (line.trim().startsWith("Name")) {
							isFound = line.contains(adapterName);
						}
					} else {
						if (isFound && line.trim().startsWith("Signal")) {
							return Integer.parseInt(
									line.split(":")[1].trim().replace("%", ""));
						}
					}
				}
			} catch (Exception ex) {
				System.out.println("getSignalStrength("
						+ adapterName + ") failed.\n" + ex);
			}
			return -1; // adapter not found or disconnected
		}

		/**
		 * Gets the name of the system network adapter for a given index.
		 * 
		 * @param adapterI
		 *            The adapter index (>=0). Typically, a system will have at
		 *            least one adapter.
		 * @return The adapter system name. Null if none at that index.
		 */
		public static String getAdapterName(int adapterI) {
			ArrayList<String> netAdapters = new ArrayList<>();
			try {
				Process process = Runtime.getRuntime()
						.exec("netsh wlan show interfaces");
				BufferedReader reader = new BufferedReader(
						new InputStreamReader(process.getInputStream()));

				// scan lines for all adapter names
				String line;
				while ((line = reader.readLine()) != null) {
					if (line.trim().startsWith("Name")) {
						String adapterName = line.split(":")[1].trim();
						netAdapters.add(adapterName);
					}
				}

				// if enough adapters return name
				if (netAdapters.size() > adapterI) {
					return netAdapters.get(adapterI);
				}
			} catch (Exception ex) {
				System.out.println("getAdapterName() failed.\n" + ex);
			}
			return null; // not found
		}
	}
}
