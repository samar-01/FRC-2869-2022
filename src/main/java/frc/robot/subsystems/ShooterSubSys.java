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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
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
	static final double tarspeed = 10000;

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

	public static void init() {
		initfalc(leftfal);
		initfalc(rightfal);
		System.out.println("INIT");
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

	private static void setFalc(double speed) {
		// leftfal.set(TalonFXControlMode.Velocity, speed);
		// rightfal.set(TalonFXControlMode.Velocity, speed);

	}

	public static void stop() {
		setFalc(0);
	}

	static final double lim775 = 0.15;
	static final double falconlim = 0.25;
	static double falconfast = 0.6;

	public static void run() {
		//Gets the value from the SmartDashboard
		SmartDashboard.getNumber("falcpower", falconfast);
		// SmartDashboard.getNumber("vel", calcVel(560, 60));

		//If A is pressed INTAKE
		if (xbox.getAButton()) {
			//Set all motors to the intake speed
			left775.set(TalonSRXControlMode.PercentOutput, lim775);
			right775.set(TalonSRXControlMode.PercentOutput, -lim775);
			leftfal.set(ControlMode.PercentOutput, falconlim);
			rightfal.set(ControlMode.PercentOutput, -falconlim);
			SmartDashboard.putBoolean("revup", false);
		} else if (xbox.getXButton()) {
			//If X is pressed Set Falcons to shooting speed
			// leftfal.set(ControlMode.PercentOutput, falconfast * xbox.getLeftY());
			// rightfal.set(ControlMode.PercentOutput, -falconfast * xbox.getLeftY());
			// setFalc(10000);

			// double lerror = tarspeed - leftfal.getSensorCollection().getIntegratedSensorVelocity();
			// leftfal.set(ControlMode.PercentOutput, clamp(lpid.calculate(lerror), -falconfast, falconfast));
			// double rerror = tarspeed - rightfal.getSensorCollection().getIntegratedSensorVelocity();
			// rightfal.set(ControlMode.PercentOutput, clamp(rpid.calculate(-rerror), -falconfast, falconfast));

			// leftfal.set(ControlMode.PercentOutput, xbox.getLeftY());
			// rightfal.set(ControlMode.PercentOutput, -xbox.getLeftY());

			/**
			 * Convert 2000 RPM to units / 100ms.
			 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			// double targetVelocity_UnitsPer100ms = (calcVel(distance(), getAngle()) *  600.0 / 2048.0) * 10 * 2048.0 / 600.0;
			
			//gives the target velocity from the Function
			// double targetVelocity_UnitsPer100ms = calcVel(distance(), getAngle()) * 2048.0 / 600.0;
			double targetVelocity_UnitsPer100ms = calcVel(distance(), getAngle());
			/* 2000 RPM in either direction */
			leftfal.set(TalonFXControlMode.Velocity, -targetVelocity_UnitsPer100ms);
			rightfal.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
			// rightfal.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

			SmartDashboard.putBoolean("revup", true);
			if (xbox.getBButton()) {
				// left775.set(TalonSRXControlMode.PercentOutput, -1);
				// right775.set(TalonSRXControlMode.PercentOutput, 1);
				left775.set(TalonSRXControlMode.PercentOutput, xbox.getLeftY()*0.25);
				right775.set(TalonSRXControlMode.PercentOutput, -xbox.getLeftY()*0.25);
				// leftfal.set(ControlMode.PercentOutput, falconfast * xbox.getLeftY());
				// rightfal.set(ControlMode.PercentOutput, -falconfast * xbox.getLeftY());}
			}
		} else {
			SmartDashboard.putBoolean("revup", false);
			left775.set(TalonSRXControlMode.PercentOutput, xbox.getLeftY()*0.5);
			right775.set(TalonSRXControlMode.PercentOutput, -xbox.getLeftY()*0.5);
			leftfal.set(ControlMode.PercentOutput, falconfast * xbox.getLeftY());
			rightfal.set(ControlMode.PercentOutput, -falconfast * xbox.getLeftY());
		}
		SmartDashboard.putNumber("leftfalconspeed", leftfal.getSensorCollection().getIntegratedSensorVelocity());
		SmartDashboard.putNumber("rightfalconspeed", rightfal.getSensorCollection().getIntegratedSensorVelocity());
		// SmartDashboard.putNumber("leftfalconpos",
		// leftfal.getSensorCollection().getIntegratedSensorPosition());
		// SmartDashboard.putNumber("target speed", 10*(calcVel(distance(), getAngle()) *  600.0 / 2048.0));
		SmartDashboard.putNumber("target speed1", (calcVel(distance(), getAngle())));
		// rightfal.getSensorCollection().getIntegratedSensorPosition());
		
		// System.out.println(calcVel(distance(), 47));
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

	/**
	 * @param d in m
	 * @param theta in degrees
	 * @return rpm
	 */
	public static double calcVel(double d, double theta){
		// tinyurl.com/projmath
		//setup numbers
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
		vel *= 7.9;
		// System.out.println(vel);
		return vel;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
