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
	double zero = 10;
	static boolean init = false;

	/**
	 * Resets the encoders of the passed motor and the passed pid controller
	 * 
	 * @param motor
	 * @param pid
	 */
	void resetEnc(CANSparkMax motor, PIDController pid) {
		motor.getEncoder().setPosition(0);
		pid.reset();
	}

	/**
	 * Calls resetEnc for the left and right motors and the lPID and rPID
	 * controllers
	 */
	void resetEnc() {
		resetEnc(left, lPID);
		resetEnc(right, lPID);
	}

	/**
	 * Moves the passed motor with the passed pid controller to
	 * ClimberSubSystem.target and clamps to 0.6 when going down and 1 when going up
	 * 
	 * @param motor motor to move
	 * @param pid pid controller to use
	 */
	void pidmove(CANSparkMax motor, PIDController pid) {
		double error = target - getPos(motor);
		double power = pid.calculate(-error);
		if (power > 0) {
			power = clamp(power, -0.6, 0.6);
		} else {
			power = clamp(power, -1, 1);
		}
		// System.out.println(power);
		motor.set(power);
	}

	// Umesh was here :)
	// Who is Umesh
	// Why is Umesh

	/**
	 * Moves both left and right motors using the PID Controllers lPID and rPID
	 */
	void pidmove() {
		pidmove(left, lPID);
		pidmove(right, rPID);
	}

	/**
	 * On startup reset the climbers's position and set the encoder to 0
	 * Also inverts the right side so both motors spin the same way
	 * Can be called many times, only happens once
	 * to call again set ClimberSubSys.init to false
	 */
	public void init() {
		if (!init) {
			resetEnc();
			right.setInverted(true);
			init = true;
		}
	}

	/**
	 * moves up
	 */
	public void moveUp() {
		set(1);
	}

	/**
	 * moves down with checking for limit switch
	 */
	public void moveDown() {
		set(-1);
		resetOnSwitch();
	}

	/**
	 * calls resetOnSwitch
	 * 
	 * @return true when both climbers are calibrated at the limit switches
	 */
	public boolean isCalib() {
		resetOnSwitch();
		return !leftswitch.get() && !rightswitch.get();
	}

	/**
	 * sets both motors to the same passed speed
	 * does not allow the motors to go if the limit switch is pressed or if it would
	 * go too high up (uplim)
	 * 
	 * @param speed the speed to set the motors
	 */
	void set(double speed) {
		if (speed == 0) {
			left.set(0);
			right.set(0);
			return;
		}
		if (leftswitch.get() && speed < 0) {
			left.set(speed * speedlim);
		} else if (getPos(left) < uplim && speed > 0) {
			left.set(speed * speedlim);
		} else {
			left.set(0);
		}
		if (rightswitch.get() && speed < 0) {
			right.set(speed * speedlim);
		} else if (getPos(right) < uplim && speed > 0) {
			right.set(speed * speedlim);
		} else {
			right.set(0);
		}
	}

	/**
	 * gets the position of the passed motor
	 * 
	 * @param motor the motor to get the position of
	 * @return the encoder position
	 */
	double getPos(CANSparkMax motor) {
		return motor.getEncoder().getPosition();
	}

	/**
	 * Sets the target to the correct position based on the dpad on the main
	 * controller or the operator controller
	 * if operator's controller has start button pressed it will be in manual mode
	 * and set the motors to 0.1 power
	 */
	public void run() {
		// SmartDashboard.putNumber("leftclimb", left.getEncoder().getPosition());
		// SmartDashboard.putNumber("rightclimb", right.getEncoder().getPosition());
		// System.out.println(opxbox.getPOV());

		// if (xbox.getRightStickButtonPressed()){
		// resetEnc();
		// }

		int xpov = xbox.getPOV();
		// if (xpov == 270){
		// resetEnc(left, lPID);
		// } else if (xpov == 90){
		// resetEnc(right, rPID);
		// }

		if (xpov == 0) {
			target = 220;
			pidmove();
		} else if (xpov == 180) {
			target = zero;
			pidmove();
		} else if (xpov == -1) {
			int pov = opxbox.getPOV();
			if (pov == -1) {
				set(0);
			}
			if (pov == 0) {
				if (opxbox.getStartButton()) {
					left.set(0.1);
					right.set(0.1);
				} else {
					moveUp();
				}
			}
			if (pov == 180) {
				if (opxbox.getStartButton()) {
					left.set(-0.1);
					right.set(-0.1);
				} else {
					moveDown();
				}
			}
		} else {
			set(0);
		}

		// resetOnSwitch();
	}

	/**
	 * if the limit switches are pressed, it will call resetEnc of that motor
	 */
	void resetOnSwitch() {
		if (!leftswitch.get()) {
			resetEnc(left, lPID);
		}
		if (!rightswitch.get()) {
			resetEnc(right, rPID);
		}
	}

	/** Creates a new Climber. */
	public ClimberSubSys() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
