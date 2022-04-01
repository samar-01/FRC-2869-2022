// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class AngleTo extends CommandBase {
	AngleSubSys angleSubSys;
	double target;
	/** Creates a new AngleTo. */
	public AngleTo(double d) {
		this.angleSubSys = RobotContainer.angleSubSys;
		target = d;
		addRequirements(angleSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		angleSubSys.init();
		angleSubSys.resetPID();
		angleSubSys.setTarget(target);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		angleSubSys.pidmove();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		angleSubSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return angleSubSys.isLifted();
	}
}
