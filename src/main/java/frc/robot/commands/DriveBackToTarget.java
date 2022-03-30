// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubSys;
import static frc.robot.Constants.*;

public class DriveBackToTarget extends CommandBase {
	DrivetrainSubSys drive;
	double [] power;
	/** Creates a new DriveBackToTarget. */
	public DriveBackToTarget() {
		drive = RobotContainer.drivetrainSubSys;
		power = new double[2];
		addRequirements(drive);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		double lrat = distancedriven[0] / (distancedriven[0] + distancedriven[1]);
		double rrat = distancedriven[1] / (distancedriven[0] + distancedriven[1]);
		power[0] = lrat * powerRev;
		power[1] = rrat * powerRev;
		drive.resetEncoders();
	}

	boolean close(double a, double b){
		return Math.abs(a - b) < 1;
	}
	boolean done = false;
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (close(drive.getenc()[0], -distancedriven[0]) || close(drive.getenc()[1], -distancedriven[1])){
			done = true;
		} else {
			drive.left.set(-power[0]);
			drive.right.set(-power[1]);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return done;
	}
}
