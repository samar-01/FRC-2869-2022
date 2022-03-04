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
	static double kp = 0.01, ki = 0.001, kd = 0.005, tolerance = 10;
	static PIDController armPID = new PIDController(kp, ki, kd);
	static final double target = 120, limit = 135;
	public static void init() {
		arm.getEncoder().setPosition(0);
		armPID.reset();
		armPID.setPID(kp, ki, kd);
		armPID.setTolerance(1);
	}

	public void run() {
		if (getEncAngle() > limit){
			arm.set(0);
		} else if (getEncAngle() < -2){
			arm.set(0);
			// TODO it is on brake mode so it will stay in teh rubber so moe code is needed
		} else {
			if (!opxbox.getYButton()) {
				arm.set(armlim * (opxbox.getRightTriggerAxis() - opxbox.getLeftTriggerAxis()));
				// arm.set(armlim * (xbox.getRightTriggerAxis() - xbox.getLeftTriggerAxis()));
				armPID.reset();
			} else {
				double error = target - getEncAngle();
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

	public static double getAngle() {
		return arm.getEncoder().getPosition()*0.72340426-34;
		// return 60;
	}
	public static double getEncAngle() {
		return arm.getEncoder().getPosition();
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
