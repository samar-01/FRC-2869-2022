// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubSys extends SubsystemBase {
	/** Creates a new ShooterSubSys. */
	public ShooterSubSys() {
	}

	private static final WPI_TalonFX leftfal = new WPI_TalonFX(8);
	private static final WPI_TalonFX rightfal = new WPI_TalonFX(7);
	private static final WPI_TalonSRX left775 = new WPI_TalonSRX(10);
	private static final WPI_TalonSRX right775 = new WPI_TalonSRX(9);

	/**
	 * Sets up the PID Controllers on the Falcon
	 * 
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
		_talon.config_kF(Constants.kPIDLoopIdx, Constants.falconGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, Constants.falconGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, Constants.falconGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, Constants.falconGains.kD, Constants.kTimeoutMs);
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

	public static void init() {
		leftfal.setNeutralMode(NeutralMode.Coast);
		rightfal.setNeutralMode(NeutralMode.Coast);
		initfalc(leftfal);
		initfalc(rightfal);

		// + is shoot - is intake
		leftfal.setInverted(true);
		rightfal.setInverted(false);
		left775.setInverted(false);
		right775.setInverted(true);
		// lpid.setPID(kp, ki, kd);
		// rpid.setPID(kp, ki, kd);
		// lpid.setTolerance(100);
		// rpid.setTolerance(100);
	}

	private static void setFalcPID(double speed) {
		leftfal.set(TalonFXControlMode.Velocity, speed);
		rightfal.set(TalonFXControlMode.Velocity, speed);
	}

	public static void stop() {
		setFalcPID(0);
	}

	static final double lim775 = 0.15;
	static final double falconInLim = 0.25;
	static double falconFastLim = 0.6;

	static void PIDSpeed(double speed) {
		double targetVelocity_UnitsPer100ms = speed;
		leftfal.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
		rightfal.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
	}

	void ramp(){
		PIDSpeed(calcVel(distance(), getAngle()));
	}

	void shoot775(){
		left775.set(TalonSRXControlMode.PercentOutput, 9.0 / RobotController.getBatteryVoltage());
		right775.set(TalonSRXControlMode.PercentOutput, 9.0 / RobotController.getBatteryVoltage());
	}

	public void run() {
		// If A is pressed INTAKE
		if (xbox.getAButton()) {
			// Set all motors to the intake speed
			left775.set(TalonSRXControlMode.PercentOutput, lim775);
			right775.set(TalonSRXControlMode.PercentOutput, lim775);
			leftfal.set(ControlMode.PercentOutput, -falconInLim);
			rightfal.set(ControlMode.PercentOutput, -falconInLim);
			SmartDashboard.putBoolean("revup", false);
		} else if (xbox.getXButton()) {
			ramp();
			SmartDashboard.putBoolean("revup", true);
			if (xbox.getBButton()) {
				shoot775();
			}
		} else {
			SmartDashboard.putBoolean("revup", false);
			left775.set(TalonSRXControlMode.PercentOutput, xbox.getRightY() * 0.5);
			right775.set(TalonSRXControlMode.PercentOutput, xbox.getRightY() * 0.5);
			leftfal.set(ControlMode.PercentOutput, falconFastLim * xbox.getRightY());
			rightfal.set(ControlMode.PercentOutput, falconFastLim * xbox.getRightY());
		}
		SmartDashboard.putBoolean("ready to shoot", isRightSpeed());
		SmartDashboard.putNumber("leftfalconspeed", leftfal.getSensorCollection().getIntegratedSensorVelocity());
		SmartDashboard.putNumber("rightfalconspeed", rightfal.getSensorCollection().getIntegratedSensorVelocity());
		SmartDashboard.putNumber("target speed", (calcVel(distance(), getAngle())));

	}

	static double h = 0.52;
	static double H = 2.64;
	static double r = 0.62;
	static double g = 9.8;
	static double wheelr = 3 * 25.4 / 1000;

	/**
	 * @param d     in m
	 * @param theta in degrees
	 * @return rps
	 */
	public static double oldCalcVel(double d, double theta) {
		theta = Math.toRadians(theta);
		double h1 = h + r * Math.sin(theta);
		double dx = d - r * Math.cos(theta);
		double dy = H - h1;
		double num = g * Math.pow(dx, 2);
		double denom = 2 * Math.pow(Math.cos(theta), 2) * (Math.tan(theta) * dx - dy);
		// double denom = 2*(dy-Math.sin(theta)*dx);
		double vel = Math.sqrt(num / denom); // m/s
		vel /= wheelr;
		vel /= (2 * Math.PI);
		vel *= 60;
		vel *= 0.7; // correct for energy loss
		return vel;
	}

	/**
	 * @param d     in m
	 * @param theta in degrees
	 * @return encoder per 100ms
	 */
	public static double calcVel(double d, double theta) {
		// tinyurl.com/projmath
		// setup numbers
		theta = Math.toRadians(theta);
		double h1 = h + r * Math.sin(theta);
		double dx = d - r * Math.cos(theta);
		double dy = H - h1;
		// below is just the equation
		double num = Math.sqrt(g) * Math.sqrt(dx) * Math.sqrt(Math.pow(Math.tan(theta), 2));
		double denom = Math.sqrt(2 * Math.tan(theta) - 2 * dy / dx);
		double vel = num / denom;
		// convert to rpm
		vel /= wheelr;
		vel /= (2 * Math.PI);
		vel *= 60;
		// correct for real life;
		vel *= 7.5;
		// 7.5 for 12.3v standby
		// 8.36 for 11.4v standy
		// System.out.println(vel);
		return vel;
	}

	double speedTol = 100;
	boolean isRightSpeed() {
		return Math.abs(calcVel(distance(), getAngle()) - leftfal.getSensorCollection().getIntegratedSensorVelocity()) <= speedTol
			&& Math.abs(calcVel(distance(), getAngle()) - rightfal.getSensorCollection().getIntegratedSensorVelocity()) <= speedTol;
	}

	double baseline = 2000;
	Timer pidready = new Timer();

	public void autoShootSpeed(AngleSubSys angleSubSys) {
		if (angleSubSys.isLifted()) {
			ramp();
			if (pidready.get() == 0 && isRightSpeed()){
				pidready.start();
			} else if (pidready.hasElapsed(1) && isRightSpeed()){
				shoot775();
			}
		} else {
			PIDSpeed(baseline);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
