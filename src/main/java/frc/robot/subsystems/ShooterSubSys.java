// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.sql.Time;

import frc.robot.Constants;

public class ShooterSubSys extends SubsystemBase {
	/** Creates a new ShooterSubSys. */
	public ShooterSubSys() {
	}

	private static final WPI_TalonFX leftfal = new WPI_TalonFX(8);
	private static final WPI_TalonFX rightfal = new WPI_TalonFX(7);
	private static final WPI_TalonSRX left775 = new WPI_TalonSRX(10);
	private static final WPI_TalonSRX right775 = new WPI_TalonSRX(9);
	static final double kp = 0.001, ki = 0.0000, kd = 0.00001;
	private static final PIDController lpid = new PIDController(kp, ki, kd);
	private static final PIDController rpid = new PIDController(kp, ki, kd);

	/**
	 * Sets up the PID Controllers on the Falcon
	 * @param _talon The Falcon TalonFX object
	 */
	static void initfalc(WPI_TalonFX _talon) {
		_talon.set(ControlMode.PercentOutput, 0);
		_talon.setNeutralMode(NeutralMode.Coast);

		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
				Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* Config the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		_talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is
		 * integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
		 * sensor-phase
		 */
		// _talon.setSensorPhase(true);
	}

	static boolean init = false;

	public static void init() {
		if (!init){
			// Shuffleboard.getTab("Teleop").addPersistent("velconstant", 7.77);
			initfalc(leftfal);
			initfalc(rightfal);
			// System.out.println("INIT");
			init = true;
		}
		// leftfal.set(ControlMode.PercentOutput, 0);
		// rightfal.set(ControlMode.PercentOutput, 0);
		// lpid.reset();
		// rpid.reset();
		// lpid.setPID(kp, ki, kd);
		// rpid.setPID(kp, ki, kd);
		// lpid.setTolerance(100);
		// rpid.setTolerance(100);
		// leftfal.setNeutralMode(NeutralMode.Coast);
		// rightfal.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * stop all shooter motors
	 */
	public void stop() {
		PIDSpeed(0);
		left775.set(ControlMode.PercentOutput, 0);
		right775.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * @return if falcons are at the pid speed
	 */
	public static boolean isAtSpeed(){
		return Math.abs((-leftfal.getSensorCollection().getIntegratedSensorVelocity() + rightfal.getSensorCollection().getIntegratedSensorVelocity())/2 - tarspeed) < 100;
	}

	static final double lim775 = 0.30;
	static final double falconInLim = 0.25;
	static double falconfast = 0.2;
	static double tarspeed = 0;

	public static double getPIDSpeed(){
		return tarspeed;
	}

	public static void PIDSpeed(double speed){
		tarspeed = speed;
		leftfal.set(TalonFXControlMode.Velocity, -tarspeed);
		rightfal.set(TalonFXControlMode.Velocity, tarspeed);
	}

	public static void rev(){
		PIDSpeed(7000);
	}

	public void autoRev(){
		PIDSpeed(calcVel());
	}

	public void shoot(){
		left775.set(TalonSRXControlMode.PercentOutput, -9.0/RobotController.getBatteryVoltage());
		right775.set(TalonSRXControlMode.PercentOutput, 9.0/RobotController.getBatteryVoltage());
	}

	static DigitalInput intake = new DigitalInput(2);

	public static boolean isIntakeEmpty(){
		return intake.get();
	}

	public void intake(){
		left775.set(TalonSRXControlMode.PercentOutput, lim775);
		right775.set(TalonSRXControlMode.PercentOutput, -lim775);
		leftfal.set(ControlMode.PercentOutput, falconInLim);
		rightfal.set(ControlMode.PercentOutput, -falconInLim);
	}

	public void run() {
		revedEntry.setBoolean(isAtSpeed());
		//Gets the value from the SmartDashboard
		SmartDashboard.getNumber("falcpower", falconfast);
		// SmartDashboard.putBoolean("intaked", !intake.get());
		// System.out.println(intake.get());
		// SmartDashboard.getNumber("vel", calcVel(560, 60));

		//If A is pressed INTAKE
		if (isIntakeEmpty() && (opxbox.getXButton() || xbox.getAButton())) {
			//Set all motors to the intake speed
			// SmartDashboard.putBoolean("revup", false);
			intake();
		} else if (xbox.getXButton()) {
			autoRev();
			if (xbox.getBButton()) {
				// left775.set(TalonSRXControlMode.PercentOutput, -1);
				// right775.set(TalonSRXControlMode.PercentOutput, 1);
				shoot();
				// leftfal.set(ControlMode.PercentOutput, falconfast * xbox.getLeftY());
				// rightfal.set(ControlMode.PercentOutput, -falconfast * xbox.getLeftY());}
			}
			
		} else if (opxbox.getRightBumper()){
			left775.set(TalonSRXControlMode.PercentOutput, 1);
			right775.set(TalonSRXControlMode.PercentOutput, -1);
			leftfal.set(ControlMode.PercentOutput, 1);
			rightfal.set(ControlMode.PercentOutput, -1);
		} else if (opxbox.getLeftBumper()){
			left775.set(TalonSRXControlMode.PercentOutput, -1);
			right775.set(TalonSRXControlMode.PercentOutput, 1);
			leftfal.set(ControlMode.PercentOutput, -1);
			rightfal.set(ControlMode.PercentOutput, 1);
		} else {
			left775.set(TalonSRXControlMode.PercentOutput, xbox.getRightY()*1);
			right775.set(TalonSRXControlMode.PercentOutput, -xbox.getRightY()*1);
			leftfal.set(ControlMode.PercentOutput, falconfast * xbox.getRightY());
			rightfal.set(ControlMode.PercentOutput, -falconfast * xbox.getRightY());
		}
		if (opxbox.getPOV() == 90){
			offFlash();
		}
		else if (opxbox.getPOV() == 270){
			onFlash();
		}
		leftFalEntry.setNumber(leftfal.getSensorCollection().getIntegratedSensorVelocity());
		rightFalEntry.setNumber(rightfal.getSensorCollection().getIntegratedSensorVelocity());
		tarSpeedEntry.setNumber(calcVel());
		// SmartDashboard.putNumber("target speed1", (calcVel()));
		// SmartDashboard.putNumber("leftfalconspeed", leftfal.getSensorCollection().getIntegratedSensorVelocity());
		// SmartDashboard.putNumber("rightfalconspeed", rightfal.getSensorCollection().getIntegratedSensorVelocity());
		// SmartDashboard.putNumber("leftfalconpos",
		// leftfal.getSensorCollection().getIntegratedSensorPosition());
		// SmartDashboard.putNumber("target speed", 10*(calcVel(distance(), getAngle()) *  600.0 / 2048.0));
		// SmartDashboard.putNumber("lfalctemp", leftfal.getTemperature());
		// SmartDashboard.putNumber("rfalctemp", rightfal.getTemperature());
		// rightfal.getSensorCollection().getIntegratedSensorPosition());
		
		// System.out.println(calcVel(distance(), 47));
		// System.out.println(left775.getSupplyCurrent());
	}

	static double h = 0.52;
	static double H = 2.64;
	static double r = 0.62;
	static double g = 9.8;
	static double wheelr = 3 * 25.4 / 1000;

	/**
	 * @param d in m
	 * @param theta in degrees
	 * @return rps
	 */
	public static double oldCalcVel(double d, double theta){
		theta = Math.toRadians(theta);
		double h1 = h + r * Math.sin(theta);
		double dx = d - r * Math.cos(theta);
		double dy = H - h1;
		double num = g * Math.pow(dx, 2);
		double denom = 2 * Math.pow(Math.cos(theta), 2) * (Math.tan(theta) * dx - dy);
		// double denom = 2*(dy-Math.sin(theta)*dx);
		double vel = Math.sqrt(num/denom); // m/s
		vel /= wheelr;
		vel /= (2*Math.PI);
		vel *= 60;
		vel *= 0.7; // correct for energy loss
		return vel;
	}

	static double thing = 7.77;
	/**
	 * @param d in m
	 * @param theta in degrees
	 * @return rpm
	 */
	public static double calcVel(){
		// tinyurl.com/projmath
		//setup numbers
		if (xbox.getPOV() == 90){
			return 7000; // TODO fix for launchpad speed
		}
		double d = distance();
		double theta = getAngle();
		theta = Math.toRadians(theta);
		double h1 = h + r * Math.sin(theta);
		double dx = d - r * Math.cos(theta);
		double dy = H - h1;
		//below is just the equation
		double num = Math.sqrt(g) * Math.sqrt(dx) * Math.sqrt(Math.pow(Math.tan(theta), 2));
		double denom = Math.sqrt(2*Math.tan(theta) - 2*dy/dx);
		double vel = num/denom;
		//convert to rpm
		vel /= wheelr;
		vel /= (2*Math.PI);
		vel *= 60;
		//correct for real life;
		// thing = SmartDashboard.getNumber("velconstant", 7.2);
		thing = SmartDashboard.getNumber("velconstant", thing);
		vel *= thing;
		// 9v constant - 

		// 7.5 for 12.3v standby
		// 8.36 for 11.4v standy
		// System.out.println(vel);
		return vel;
	}

	double baseline = 2000;
	public void autoShootSpeed(AngleSubSys angleSubSys){
		if (angleSubSys.isLifted()){
			calcVel();
		} else {
			PIDSpeed(baseline);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
