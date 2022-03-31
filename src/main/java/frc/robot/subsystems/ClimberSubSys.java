// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Inputs.*;

public class ClimberSubSys extends SubsystemBase {
	CANSparkMax left = new CANSparkMax(6, MotorType.kBrushless);
	CANSparkMax right = new CANSparkMax(11, MotorType.kBrushless);
	DigitalInput leftswitch = new DigitalInput(1);
	DigitalInput rightswitch = new DigitalInput(0);
	static double kp = 0.1, ki = 0.0, kd = 0.0, tolerance = 1;
	static PIDController lPID = new PIDController(kp, ki, kd);
	static PIDController rPID = new PIDController(kp, ki, kd);
	double target = 0;
	double speedlim = 0.6;
	double uplim = 300;
	double tarhigh = 220;
	double zero = 10;
	static boolean init = false;

	void resetEnc(CANSparkMax motor, PIDController pid){
		motor.getEncoder().setPosition(0);
		pid.reset();
	}

	void resetEnc(){
		resetEnc(left, lPID);
		resetEnc(right, lPID);
	}

	void pidmove(CANSparkMax motor, PIDController pid){
		double error = target - getPos(motor);
		double power = pid.calculate(-error);
		if (power > 0){
			power = clamp(power, -0.6, 0.6);
		} else {
			power = clamp(power, -1, 1);
		}
		// System.out.println(power);
		motor.set(power);
	}

	void pidmove(){
		pidmove(left, lPID);
		pidmove(right, rPID);
	}

	public void init() {
		if (!init){
			resetEnc();
			right.setInverted(true);
			init = true;
		}
	}

	public void moveUp(){
		set(1);
	}

	public void moveDown(){
		moveDownNoSwitch();
		resetOnSwitch();
	}

	public void moveDownNoSwitch(){
		set(-1);
	}

	public boolean isCalib(){
		resetOnSwitch();
		return !leftswitch.get() && !rightswitch.get();
	}

	void set(double speed){
		if (speed == 0){
			left.set(0);
			right.set(0);
			return;
		}
		if (leftswitch.get() && speed < 0){
			left.set(speed * speedlim);
		} else if (getPos(left) < uplim && speed > 0){
			left.set(speed * speedlim);
		} else {
			left.set(0);
		}
		if (rightswitch.get() && speed < 0){
			right.set(speed * speedlim);
		} else if (getPos(right) < uplim && speed > 0){
			right.set(speed * speedlim);
		} else {
			right.set(0);
		}
	}

	double getPos(CANSparkMax motor){
		return motor.getEncoder().getPosition();
	}

	void manualMove(double d){
		left.set(d);
		right.set(d);
	}
 
	public void run(){
		// SmartDashboard.putNumber("leftclimb", left.getEncoder().getPosition());
		// SmartDashboard.putNumber("rightclimb", right.getEncoder().getPosition());
		// System.out.println(opxbox.getPOV());
		
		// if (xbox.getRightStickButtonPressed()){
		// 	resetEnc();
		// }

		// if (xpov == 270){
		// 	resetEnc(left, lPID);
		// } else if (xpov == 90){
		// 	resetEnc(right, rPID);
		// }

		if (pidClimbUp()){
			target = tarhigh;
			pidmove();
		} else if (pidClimbDown()){
			target = zero;
			pidmove();
		} else {
			if (climbUp()){
				if (getOpOverride()){
					manualMove(manClimbSpeed);
				} else {
					moveUp();
				}
			} else if (climbDown()){
				if (getOpOverride()){
					manualMove(-manClimbSpeed);
				} else {
					moveDown();
				}
			}
			else {
				set(0);
			}
	
		} 
		// resetOnSwitch();
	}

	void resetOnSwitch(){
		if (!leftswitch.get()){
			resetEnc(left, lPID);
		}
		if (!rightswitch.get()){
			resetEnc(right, rPID);
		}
	}

	/** Creates a new Climber. */
	public ClimberSubSys() {}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
