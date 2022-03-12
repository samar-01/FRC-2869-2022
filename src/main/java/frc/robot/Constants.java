/*
Pre match checklist:
Falcon shafts collars tight
falcon shaft tight
angler left right
check bearings dont fall into water cut plate when resting down
check barstock bend
missing churro on right side
*/

/*
TODO
Auto
*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.logging.*;
import java.util.logging.FileHandler;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.*;

/*
Driver
rt - forward
lt - backwards
l stick - turn left and right 
lb - 180 (NOT GOOD DONT USE)
rb - boost
y - align
x - ramp
b - shoot (x and b need to be held together to be shot)
r stick - manual control / low goal
when hub is fully in view of photonvision then low goal can be shot at full power
start reset angle (ONLY FOR EMERGENCIES)

Operator
a - high goal
x - intake
y - low goal
b - lower arm
rt - up angle
lt - down angle
lb - eject
rb - force in
start - full manual control of angle (NO SAFETY SO EMERGENCY ONLY)
*/

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final String logfile = "/home/lvuser/Logs/log";
	public static FileHandler logFile;
	public static Logger logger = Logger.getGlobal();

	void initLogger(String startFileName) {
		if (startFileName == null) {
			startFileName = "./log0";
		}
		java.util.Date today = new java.util.Date();
		String filename = startFileName + today.getDay() + "-" + today.getHours() + "-" + today.getMinutes();
		try {
			logFile = new FileHandler(filename);
			logFile.setFormatter(new SimpleFormatter());
			logger = Logger.getGlobal();
			logger.addHandler(logFile);
			logger.setLevel(Level.INFO);
			logger.log(Level.INFO, "Logging Messages:");

		} catch (Exception e) {
			System.out.println("Unable to init log file:" + filename);
			System.out.println(e);
		}

	}

	public static void log(Level l, String s) {
		logger.log(l, s);
	}

	public static void log(String s) {
		logger.log(Level.INFO, s);
	}

	public static final XboxController xbox = new XboxController(0);
	public static final XboxController opxbox = new XboxController(1);
	public static final Joystick xboxjoystick = new Joystick(0);
	public static final JoystickButton spinDriveButton = new JoystickButton(xboxjoystick, 5); // lb
	// public static final JoystickButton autoDriveButton = new JoystickButton(xboxjoystick, 3); // X
	// public static final JoystickButton resetDriveButton = new JoystickButton(xboxjoystick, 2); // B
	// public static final JoystickButton stopDriveButton = new JoystickButton(xboxjoystick, 1); // A
	// public static final JoystickButton driveDriveButton = new JoystickButton(xboxjoystick, 5); // LB
	public static final AnalogInput ultra = new AnalogInput(2);
	private static double targetAngle = 0;

	public static void autoSchedule(Command comm) {
		if (!comm.isScheduled()) {
			comm.schedule();
		}
	}

	public static double distance() {
		return LimelightSubSys.getDistance();
	}

	public static void setAngle(double angle) {
		targetAngle = angle;
	}

	public static double getTarAngle() {
		return targetAngle;
	}

	public static double getAngle() {
		return AngleSubSys.getAngle();
	}

	public static double ang2Enc(double theta) {
		return AngleSubSys.ang2Enc(theta);
	}

	public static double getUltra() {
		double dist = (ultra.getVoltage() * 1000) / 9.77;
		SmartDashboard.putNumber("ultrasonic", dist);
		return dist;
	}

	public static double clamp(double inp, double min, double max) {
		if (inp > max) {
			inp = max;
		}
		if (inp < min) {
			inp = min;
		}
		return inp;
	}

	private static final WPI_TalonSRX flashlight = new WPI_TalonSRX(2);
	private static boolean initFlash = false;

	public static void initFlash() {
		flashlight.configContinuousCurrentLimit(1);
		flashlight.configPeakCurrentLimit(1);
		flashlight.enableCurrentLimit(true);
		initFlash = true;

		// System.out.println("INIT");
	}

	public static void onFlash() {
		if (!initFlash) {
			initFlash();
		}

		flashlight.set((5.0 / RobotController.getBatteryVoltage()));
		// flashlight.set(0.4);
	}

	public static void offFlash() {
		if (!initFlash) {
			initFlash();
		}
		flashlight.set(0);
	}

	public static double getFlash() {
		return flashlight.get();
	}

	public static NetworkTable table;
	public static NetworkTableEntry tx;
	public static NetworkTableEntry ty;
	// public static NetworkTableEntry ta;
	public static boolean networkInit = false;

	public static void networkInit() {
		if (!networkInit) {
			try {
				table = NetworkTableInstance.getDefault().getTable("limelight");
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

				tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
				ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
				// NetworkTableEntry ta =
				// NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");
				networkInit = true;
			} catch (Exception e) {
				System.out.println("Limelight network init fail");
			}
		}
	}

	public static void onLime() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
	}

	public static void offLime() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
	}

	// BELOW HERE is copied from falcon ex code
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control
	 * loop.
	 * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity
	 * units at 100% output
	 * 
	 * kP kI kD kF Iz PeakOut
	 */
	public final static Gains kGains_Velocit = new Gains(0.1, 0.001, 10, 1023.0 / 20660.0, 300, 1.00);
}
