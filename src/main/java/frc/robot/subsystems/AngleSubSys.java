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
	static PIDController armPID = new PIDController(kp, ki, kd);
	static double target = 70, limit = 80;
	static double base = -34, max = target, mid = 45;
	// set target to real target - 10
	static boolean past = false;

	public static void init() {
		arm.getEncoder().setPosition(0);
		armPID.reset();
		armPID.setPID(kp, ki, kd);
		armPID.setTolerance(tolerance);
	}

	public void run() {
		if (getAngle() > limit) {
			arm.set(0);
		} else if (getAngle() < -36) {
			arm.set(0);
			// TODO it is on brake mode so it will stay in teh rubber so moe code is needed
		} else {
			if (!opxbox.getAButton() && !opxbox.getYButton() && !opxbox.getBButton()) {
				arm.set(armlim * (opxbox.getRightTriggerAxis() - opxbox.getLeftTriggerAxis()));
				// arm.set(armlim * (xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis()));
				armPID.reset();
			} else {
				if (opxbox.getAButton()) {
					// if (getAngle() > target && !past){
					// target += 10;
					// past = true;
					// } else if (getAngle() < 0){
					// past = false;
					// }
					target = max;
				} else if (opxbox.getYButton()){
					target = mid;
				} else if (opxbox.getBButton()){
					target = base;
				}
				double error = target - getAngle();
				double power = armPID.calculate(-error);
				power = clamp(power, -armlim, armlim);
				// System.out.println(power);
				arm.set(power);
			}
		}

		SmartDashboard.putNumber("angle", getAngle());
		SmartDashboard.putNumber("encangle", getEncAngle());
		// SmartDashboard.putNumber("angle", arm.getEncoder().getPosition());
	}

	public void lift() {
		double target = 120;
		double error = target - getEncAngle();
		double power = armPID.calculate(-error);
		power = clamp(power, -armlim, armlim);
		// System.out.println(power);
		arm.set(power);
	}

	public boolean isLifted() {
		return (Math.abs(target - getEncAngle()) < 1);
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
