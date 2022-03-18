/*
Pre match checklist
Falcon shafts collars tight
angler left right
check bearings dont fall into water cut plate when resting down
check barstock bend
missing churro on right side
grease chain

TODO
check 775 going too fast
battery voltage affecting shoot constant
*/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.*;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*
Driver
rt forward
lt backwards
l stick x 
rb boost
y align
x ramp
b shoot (x and b need to be held together to be shot)
r stick manual control / low goal
when hub is fully in view then low goal can be shot at full power

Operator
a high goal
x intake
y low goal
b lower arm
rt up angle
lt down angle
lb eject
rb force in
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
	public static final XboxController xbox = new XboxController(0);
	public static final XboxController opxbox = new XboxController(1);
	public static final Joystick xboxjoystick = new Joystick(0);
	// public static final JoystickButton autoDriveButton = new JoystickButton(xboxjoystick, 3); // X
	// public static final JoystickButton spinDriveButton = new JoystickButton(xboxjoystick, 4); // Y
	// public static final JoystickButton resetDriveButton = new JoystickButton(xboxjoystick, 2); // B
	// public static final JoystickButton stopDriveButton = new JoystickButton(xboxjoystick, 1); // A
	// public static final JoystickButton driveDriveButton = new JoystickButton(xboxjoystick, 5); // LB
	public static final JoystickButton spinDriveButton = new JoystickButton(xboxjoystick, 5); // lb
	public static final AnalogInput ultra = new AnalogInput(2);
	private static double targetAngle = 0;

	public static final PhotonCamera photon = new PhotonCamera("photonvision");

	public void initPhoton(){
		if (DriverStation.getAlliance() == Alliance.Blue){
			photon.setPipelineIndex(0);
		} else if (DriverStation.getAlliance() == Alliance.Red){
			photon.setPipelineIndex(1);
		} else {
			photon.setPipelineIndex(2);
		}	
	}
	/**
	 * 
	 * @return best target or null if not found
	 */
	static PhotonTrackedTarget pTarget(){
		PhotonPipelineResult result = photon.getLatestResult();
		if(result.hasTargets()){
			return result.getBestTarget();
		} else {
			return null;
		}
	}

	/**
	 * 
	 * @returns yaw of best ball or 0 if none found
	 */
	public static double pTrack(){
		PhotonTrackedTarget target = pTarget();
		if (target != null){
			return target.getYaw();
		} else {
			return 0;
		}
	}

	public static void autoSchedule(Command comm){
		if (!comm.isScheduled()){
			comm.schedule();
		}
	}

	public static double distance(){
		// return 18*12.0/39.37;
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
	public static double ang2Enc(double theta){
		return AngleSubSys.ang2Enc(theta);
	}

	public static double getUltra() {
		return 10;
		// double dist = (ultra.getVoltage()*1000)/9.77;
		// SmartDashboard.putNumber("ultrasonic", dist);
		// return dist;
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

	private static final WPI_TalonSRX flashlight = new WPI_TalonSRX(20);
	private static boolean initFlash = false;

	public static void initFlash() {
		flashlight.configContinuousCurrentLimit(1);
		flashlight.configPeakCurrentLimit(1);
		flashlight.enableCurrentLimit(true);
		initFlash=true;

		// System.out.println("INIT");
	}

	public static void onFlash() {
		// System.out.println("ON");
		if (!initFlash) {
			initFlash();
			// System.out.println("ON FAIL");
		}

		// System.out.println("ON SUCCESS");
		// flashlight.set((5.0/RobotController.getBatteryVoltage()));
		// flashlight.set(0.4);
	}

	public static void offFlash() {
		// System.out.println("OFF");
		if (!initFlash) {
			initFlash();
			// System.out.println("OFF FAIL");
		}
		flashlight.set(0);
		// System.out.println("OFF SUCCESS");
	}

	public static double getFlash() {
		return flashlight.get();
		// return 0;
	}

	
	static NetworkTable table;
	static NetworkTableEntry tx;
	static NetworkTableEntry ty;
	static NetworkTableEntry ta;
	static boolean networkInit = false;
	public static void networkInit(){
		// if (!networkInit){
		if (true){
			try{
				NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
				NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
				
				NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
				NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
				NetworkTableEntry ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");
				networkInit = true;
			}catch (Exception e){
				System.out.println("Network init fail");
			}
		}
	}

	public static void onLime(){
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
	}
	public static void offLime(){
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
	}


	//BELOW HERE is copied from falcon ex code
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
