// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothing1Sec extends CommandBase {
	/** Creates a new DoNothing1Sec. */
	public DoNothing1Sec() {
		System.out.print("created do nothing and scheduled is ");
		addRequirements();
		this.schedule();
		System.out.println(this.isScheduled());
	}

	Timer t = new Timer();

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		t.start();
		System.out.println("started doing nothing");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println("doing nothing for " + ((double)((int)(t.get()*100)))/100 + " seconds");
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("finished doing nothing");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return t.hasElapsed(1);
		// return false;
	}
}
