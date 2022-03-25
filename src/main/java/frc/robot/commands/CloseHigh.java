// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer; 
import static frc.robot.Constants.*;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.ShooterSubSys;

public class CloseHigh extends CommandBase {
	ShooterSubSys shooterSubSys;
	AngleSubSys angleSubSys;
	/** Creates a new CloseHigh. */
	public CloseHigh() {
		this.shooterSubSys = RobotContainer.shooterSubSys;
		this.angleSubSys = RobotContainer.angleSubSys;
		addRequirements(shooterSubSys, angleSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// shooterSubSys.highClose();
		shooterSubSys.PIDSpeed(closeVel * 0.8);
		angleSubSys.setTargetHighClose();
	}

	boolean done = false;
	Timer revtimer = new Timer();
	Timer timer = new Timer();
	double revdelay = 1, shoottime = 0.5;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		angleSubSys.pidmove();
		// if (angleSubSys.isLifted()){
		// 	done = true;
		// }
		if (shooterSubSys.isAtSpeed() && revtimer.get() == 0){
			revtimer.start();
		} else if (shooterSubSys.isAtSpeed() && timer.get() == 0 && revtimer.hasElapsed(revdelay) && angleSubSys.isLifted()){
			timer.start();
			shooterSubSys.shoot();
		} else if (timer.hasElapsed(shoottime) && revtimer.hasElapsed(revdelay)){
			done = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooterSubSys.stop();
		angleSubSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
