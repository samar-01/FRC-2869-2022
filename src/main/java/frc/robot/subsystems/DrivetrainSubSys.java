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
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
// import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
// not necessary but if something breaks uncomment below
// import frc.robot.commands.Shoot;
// import frc.robot.subsystems.ShootSubSys;
// import frc.robot.subsystems.IntakeSubSys;

public class DrivetrainSubSys extends SubsystemBase {
	private static final CANSparkMax right1 = new CANSparkMax(1, MotorType.kBrushless);
	private static final CANSparkMax right2 = new CANSparkMax(2, MotorType.kBrushless);
	private static final CANSparkMax left1 = new CANSparkMax(3, MotorType.kBrushless);
	private static final CANSparkMax left2 = new CANSparkMax(4, MotorType.kBrushless);
	private static final MotorControllerGroup right = new MotorControllerGroup(right1, right2);
	private static final MotorControllerGroup left = new MotorControllerGroup(left1, left2);
	private static final DifferentialDrive difDrive = new DifferentialDrive(left, right);
	private static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

	public DrivetrainSubSys() {
		difDrive.setSafetyEnabled(false);
	}


	public static void drv(double speed, double turn){
		difDrive.arcadeDrive(turn, speed);
	}

	public void autoInit(){
		left1.getEncoder().setPosition(0);
		left2.getEncoder().setPosition(0);
		right1.getEncoder().setPosition(0);
		right2.getEncoder().setPosition(0);
	}

	public double getEncDistance(){
		// return left1.getEncoder().getPosition();
		double encoders[] = {left1.getEncoder().getPosition(), left2.getEncoder().getPosition(), -right1.getEncoder().getPosition(), -right2.getEncoder().getPosition()};
		return average(encoders);
	}

	public static double autospeed = 0.5;

	public void auto(){
		// SmartDashboard.putNumber("test", getEncDistance());
		drv(-autospeed, 0);
	}

	static PIDController pidturn = new PIDController(0.8, 0.5, 0.07);
	// PhotonCamera photoncam = new PhotonCamera("photonvision");
	
	void autoTurn(double d){
		drv(0,clamp(pidturn.calculate(d)/10,-autoTurnSpeed,autoTurnSpeed));
	}


	double autoTurnSpeed = 0.4;	
	void autoTurn(double d, double speed){
		drv(speed,clamp(pidturn.calculate(d)/10,-autoTurnSpeed,autoTurnSpeed));
	}

	public void limeTurn(){
		// drv(0,clamp(pidturn.calculate(-LimelightSubSys.getLimeX())/10,-0.5,0.5));
		autoTurn(-LimelightSubSys.getLimeX());
	}

	public void resetTurn(){
		pidturn.reset();
	}

	boolean isBallTurn = false; //checks if first time running ballturndrive and resets pid
	/**
	 * turns towards ball
	 * */	
	public void ballTurn(){
		if (!isBallTurn){
			isBallTurn = true;
			pidturn.reset();
		}
		double track = -pTrack();
		if (track != Double.POSITIVE_INFINITY){
			autoTurn(track);
		}
	}
	/**
	 * 
	 */
	public void ballTurnDrive(){
		if (!isBallTurn){
			isBallTurn = true;
			pidturn.reset();
		}
		double track = -pTrack();
		if (track != Double.NEGATIVE_INFINITY){
			autoTurn(track,0.5);
			// drv(0,track/10);
			// System.out.println(track);
		}
	}

	public boolean isBallPointed(){
		return (pTrack() != Double.POSITIVE_INFINITY && Math.abs(pTrack()) < 2);
		// return false;
	}

	public void drive() {
		double speed = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
		if (xbox.getRightBumper()){
			speed *= 0.7;
		} else if (xbox.getLeftBumper()){
			
		} else {
			speed *= 0.45;
		}
		drv(speed, xbox.getLeftX()*0.7);
		
		if (xbox.getYButtonPressed()){
			resetTurn();
		}
		if (xbox.getYButton()){
			// System.out.println(limeturn.calculate(-LimelightSubSys.getLimeX()));
			limeTurn();
		}

		// if (xbox.getPOV() == 270){
		// 	photonBlue();
		// 	ballTurn();
		// } else if (xbox.getPOV() == 90){
		// 	photonRed();
		// 	ballTurn();
		// } else {
		// 	offFlash();
		// 	isBallTurn = false;
		// }

		// drv(0,0);

		// if (xbox.getLeftBumperPressed()){
		// 	limeturn.reset();
		// }
		// if (xbox.getLeftBumper()){
		// 	if (photoncam.getLatestResult().hasTargets()){
		// 		double dist = photoncam.getLatestResult().getBestTarget().getYaw();
		// 		drv(0,clamp(limeturn.calculate(dist/1),-0.5,0.5));
		// 	}
		// }
		// System.out.println(LimelightSubSys.getX());
		// SmartDashboard.putNumber("x", LimelightSubSys.getLimeX());

	}

	public static void limeturn(){
		drv(0,clamp(pidturn.calculate(-LimelightSubSys.getLimeX())/10,-0.5,0.5));
	}
	
	public void point(){
		drv(0,LimelightSubSys.getLimeX()/20);
	}

	public boolean isLimePointed(){
		return (LimelightSubSys.getLimeX()/20 < 0.1);
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
		// 	target = getUltra();
		// }
	}

	public void autodrv(){
		// double speed = xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis();
		// speed *= 0.4;

		// double pos = getUltra();
		// double turn = xbox.getLeftX()*0.5;
		// double kp = 0.3;
		// double speed = kp * (pos - target);
		// double maxspeed = 0.3;
		// speed = clamp(speed, -maxspeed, maxspeed);
		// drv(speed, turn);
	}
	
	double pos;
	double spinTarget;
	public boolean spun = false;
	Timer spintime = new Timer();
	double kp=0.4,ki=0.0,kd=0.1;
	PIDController spinner = new PIDController(kp,ki,kd);
	
	public void setRot(double angle){
		spun = false;
		ahrs.reset();
		pos = ahrs.getYaw();
		spinTarget = pos+angle;
		spintime.reset();
		spintime.start();
		spinner.reset();
		// kp = SmartDashboard.getNumber("kp", kp);
		// ki = SmartDashboard.getNumber("ki", ki);
		// kd = SmartDashboard.getNumber("kd", kd);
		spinner.setPID(kp, ki, kd);
		spinner.setTolerance(5);
		// spinner.enableContinuousInput(-180, 180);
	}
	
	public void setRot0(){
		spun = false;
		// ahrs.reset();
		pos = ahrs.getYaw();
		spinTarget = 0;
		spintime.reset();
		spintime.start();
		spinner.reset();
		// kp = SmartDashboard.getNumber("kp", kp);
		// ki = SmartDashboard.getNumber("ki", ki);
		// kd = SmartDashboard.getNumber("kd", kd);
		spinner.setPID(kp, ki, kd);
		spinner.setTolerance(2);
		// spinner.enableContinuousInput(-180, 180);
	}

	double[] errors = new double[10];
	int count = 0;

	public void resetPID(){
		resetTurn();
		// for (int i = 0; i < errors.length; i++){
		// 	errors[i] = 999;
		// }
	}

	public double average(double[] arr){
		// double avg = 0;
		// for (int i = 0; i < arr.length; i++){
		// 	avg += Math.abs(arr[i]/arr.length);
		// }
		// return avg;
		double sum = 0;
		for (int i = 0; i < arr.length; i++){
			sum += arr[i];
		}
		return sum/arr.length;
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
		// errors[count] = error;
		// count++;
		// if (count >= errors.length){
		// 	count = 0;
		// }
		turn = spinner.calculate(-error);
		turn = clamp(turn, -maxspeed, maxspeed);
		if (ahrs.isConnected()){
			drv(0, turn);
		} else{
			spun=true;
		}
		// SmartDashboard.putString("livepos", livepos+"");
		// SmartDashboard.putString("pos", pos+"");
		// SmartDashboard.putString("spintarget", spinTarget+"");
		// SmartDashboard.putString("error", error+"");
		// System.out.println(ahrs.getYaw());
		// if (spintime.hasElapsed(1)){
		// if (Math.abs(error) < 1 || spintime.hasElapsed(2)){
		if (spinner.atSetpoint() || spintime.hasElapsed(10)){
			spun = true;
			System.out.println("fin");
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
		target = getUltra();
		// System.out.println("RESET");
	}

	public static void ahrsReset(){
		ahrs.reset();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	double posD;
	double driveTarget;
	public boolean drove = false;
	Timer drivetime = new Timer();
	double kpd=0.4,kid=0.0,kdd=0.1;
	PIDController driver = new PIDController(kp,ki,kd);
	
	public void setDrivePID(double distance){
		drove = false;
		resetEncoders();
		posD = getEncDistance();
		driveTarget = posD+distance;
		drivetime.reset();
		drivetime.start();
		driver.reset();
		kpd = SmartDashboard.getNumber("kp", kp);
		kid = SmartDashboard.getNumber("ki", ki);
		kdd = SmartDashboard.getNumber("kd", kd);
		driver.setPID(kpd, kid, kdd);
		driver.setTolerance(5);
	}
	
	double[] errorsD = new double[10];
	int countD = 0;
	public void drivePID(){
		double livepos = left1.getEncoder().getPosition();
		double maxspeed = 0.6;
		double speed;
		double error = spinTarget-livepos;
		speed = driver.calculate(-error);
		speed = clamp(speed, -maxspeed, maxspeed);
		drv(speed,0);
		if (driver.atSetpoint() || spintime.hasElapsed(10)){
			drove = true;
			System.out.println("fin");
		}
	}

}
