// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 0 = -34
// 47 = 0

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class AngleSubSys extends SubsystemBase {
	/** Creates a new AngleSubSys. */
	public AngleSubSys() {
	}

	static final double armlim = 0.4;
	static final CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
	static final double ratio = 5 * 4 * 3 * 4;
	static double kp = 0.04, ki = 0.000, kd = 0.006, tolerance = 1;
	// TODO: Increase this tolerance value to 5 and see if we still score accurately.

	static PIDController armPID = new PIDController(kp, ki, kd);
	public static double base = -34, max = 70, mid = 55;
	public static double target = base, limit = 80;
	// set target to real target - 10
	static boolean past = false;
	static boolean init = false;

	public void stop(){
		arm.set(0);
	}

	/**
	 * Allows other systems to adjust the tolerance & control accuracy of angle.
	 * When the angle reaches a avlue within t of the desired angle, it should stop.
	 * @param t is measured in degrees.
	 */
	public void setPIDtolerance(double t) {
		tolerance = t;
		angletol = t;
	}
	
	public static void resetPID(){
		armPID.reset();
		armPID.setPID(kp, ki, kd);
		armPID.setTolerance(tolerance);
	}

	public static void init() {
		if (!init){
			arm.getEncoder().setPosition(0);
			resetPID();
			init = true;
		}
	}

	public void setPIDtolerance(){
		setPIDtolerance(angletolerance.getDouble(1));
	}

	public void setTargetHigh(){
		target = max;
		setPIDtolerance();
	}
	public void setTargetHighClose(){
		target = closeAngle;
		setPIDtolerance();
	}
	public void setTargetMid(){
		target = mid;
		setPIDtolerance();
	}
	public void setTargetLow(){
		target = base;
		setPIDtolerance();
	}

	public void setTarget(){
		if (opxbox.getAButton()) {
			setTargetHigh();
		} else if (opxbox.getYButton()){
			setTargetMid();
		} else if (opxbox.getBButton()){
			setTargetLow();
		}
	}

	public void run() {
		if (opxbox.getStartButton()){
			arm.set(armlim * (opxbox.getRightTriggerAxis() - opxbox.getLeftTriggerAxis()));
			armPID.reset();
			target = getAngle();
		} else if (xbox.getStartButtonReleased()){
			arm.getEncoder().setPosition(0);
		} else {
			if (getAngle() > limit) {
				// arm.set(0);
				target = mid;
			} else if (getAngle() < -36) {
				// arm.set(0);
				target = mid;
			} else {
				// if (!opxbox.getAButton() && !opxbox.getYButton() && !opxbox.getBButton()) {
				// 	arm.set(armlim * (opxbox.getRightTriggerAxis() - opxbox.getLeftTriggerAxis()));
				// 	// arm.set(armlim * (xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis()));
				// 	armPID.reset();
				// } else {
				// }
				setTarget();
				pidmove();
			}
		}

		angleEntry.setNumber(getAngle());
	}

	public void pidmove(){
		double error = target - getAngle();
		double power = armPID.calculate(-error);
		power = clamp(power, -armlim, armlim);
		// System.out.println(power);
		arm.set(power);
	}

	public void lift() {
		double target = max;
		pidmove();
	}

	public void descend() {
		double target = base;
		pidmove();
	}

	double liftTolerance = 2;
	public boolean isLifted() {
		return (Math.abs(target - getAngle()) < liftTolerance);
	}

	public static double getAngle() {
		return arm.getEncoder().getPosition() * 0.72340426 - 34;
		// return 60;
	}

	public static double getEncAngle() {
		return arm.getEncoder().getPosition();
	}

	public static double ang2Enc(double theta) {
		return (theta + 34) / 0.72340426;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
