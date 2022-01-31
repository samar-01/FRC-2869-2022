// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
// not necessary but if something breaks uncomment below
// import frc.robot.commands.Shoot;
// import frc.robot.subsystems.ShootSubSys;
// import frc.robot.subsystems.IntakeSubSys;

public class DrivetrainSubSys extends SubsystemBase {
	public static final CANSparkMax right1 = new CANSparkMax(1, MotorType.kBrushless);
	public static final CANSparkMax right2 = new CANSparkMax(2, MotorType.kBrushless);
	public static final CANSparkMax left1 = new CANSparkMax(3, MotorType.kBrushless);
	public static final CANSparkMax left2 = new CANSparkMax(4, MotorType.kBrushless);
	private static final MotorControllerGroup right = new MotorControllerGroup(right1, right2);
	private static final MotorControllerGroup left = new MotorControllerGroup(left1, left2);
	private static final DifferentialDrive difDrive = new DifferentialDrive(left, right);
	private static final XboxController xbox = Constants.xbox;
	private static final AHRS ahrs = new AHRS(SPI.Port.kMXP);
	public DrivetrainSubSys() {
		
	}

	public void drive() {
		double speed = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
		speed *= 0.4;
		drv(speed, Constants.xbox.getLeftX()*0.6);
	}
	
	boolean start = true;
	public static double target = 0;

	public void autoDrive(){
		autodrv();
		// if (xbox.getXButton() && !start){
		// 	autodrv();
		// } else {
		// 	drive();
		// }
		
		// if (xbox.getYButton()){
		// 	resetEncoders();
		// 	start = false;
		// 	target = Constants.getUltra();
		// }
	}

	public void autodrv(){
		double pos = Constants.getUltra();
		// double speed = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
		// speed *= 0.4;
		double turn = Constants.xbox.getLeftX()*0.5;
		double kp = 0.3;
		double speed = kp * (pos - target);
		double maxspeed = 0.3;
		speed = Constants.clamp(speed, -maxspeed, maxspeed);
		drv(speed, turn);
	}

	public static void drv(double speed, double turn){
		difDrive.arcadeDrive(turn, speed);
	}
	
	double pos;
	double spinTarget;
	public boolean spun = false;
	Timer spintime = new Timer();
	double kp=0.4,ki=0.0,kd=0.08;
	PIDController spinner = new PIDController(kp,ki,kd);
	
	public void setRot(){
		spun = false;
		ahrs.reset();
		pos = ahrs.getYaw();
		spinTarget = pos+180;
		spintime.reset();
		spintime.start();
		spinner.reset();
		kp = SmartDashboard.getNumber("kp", kp);
		ki = SmartDashboard.getNumber("ki", ki);
		kd = SmartDashboard.getNumber("kd", kd);
		spinner.setPID(kp, ki, kd);
	}
	
	public void spin(){
		double livepos = ahrs.getYaw();
		if (livepos<0){
			livepos += 360;
		}
		// double kp = 0.3;
		// double turn = kp * (spinTarget-livepos);
		double maxspeed = 0.6;
		double turn;
		double error = spinTarget-livepos;
		turn = spinner.calculate(-error);
		turn = Constants.clamp(turn, -maxspeed, maxspeed);
		drv(0, turn);
		// SmartDashboard.putString("livepos", livepos+"");
		// SmartDashboard.putString("pos", pos+"");
		// SmartDashboard.putString("spintarget", spinTarget+"");
		// SmartDashboard.putString("error", error+"");
		// System.out.println(ahrs.getYaw());
		if (Math.abs(error) < 5 || spintime.hasElapsed(2)){
			spun = true;
			// System.out.println("fin");
		}
	}

	public void stop() {
		stopDrive();
	}

	public static void stopDrive() {
		drv(0, 0);
	}

	public static void resetEncoders(){
		left1.getEncoder().setPosition(0);
		left2.getEncoder().setPosition(0);
		right1.getEncoder().setPosition(0);
		right2.getEncoder().setPosition(0);
		left1.getEncoder().setPositionConversionFactor(1);
		left2.getEncoder().setPositionConversionFactor(1);
		right1.getEncoder().setPositionConversionFactor(1);
		right2.getEncoder().setPositionConversionFactor(1);
		ahrs.reset();
		target = Constants.getUltra();
		System.out.println("RESET");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
