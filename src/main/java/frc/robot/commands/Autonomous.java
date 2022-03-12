// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.ShooterSubSys;

public class Autonomous extends CommandBase {
	DrivetrainSubSys drivetrainSubSys;
	AngleSubSys angleSubSys;
	ShooterSubSys shooterSubSys;
	/** Creates a new Autonomous. */
	public Autonomous(DrivetrainSubSys drivetrainSubSys, AngleSubSys angleSubSys, ShooterSubSys shooterSubSys) {
		this.drivetrainSubSys = drivetrainSubSys;
		this.angleSubSys = angleSubSys;
		this.shooterSubSys = shooterSubSys;
		addRequirements(drivetrainSubSys, angleSubSys, shooterSubSys);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrainSubSys.autoInit();
		angleSubSys.target = angleSubSys.max;
	}

	Timer revup = new Timer();
	double revtime = 1;
	double finaltime = revtime + 1;
	double drivedist = -40;

	boolean driven(){
		return drivetrainSubSys.getEncDistance() < drivedist;
	}
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putBoolean("autoready", false);
		
		SmartDashboard.putBoolean("lifted", angleSubSys.isLifted());
		if (angleSubSys.isLifted() && driven()){
			if (revup.get() == 0){
				revup.start();
			}
		}

		if (!driven()){
			drivetrainSubSys.auto();
			angleSubSys.lift();
			revup.stop();
			revup.reset();
			// shooterSubSys.rev();

			SmartDashboard.putString("auto state", "backing up");
		}
		else if (!angleSubSys.isLifted()){
			drivetrainSubSys.stop();
			
			SmartDashboard.putString("auto state", "lifting");
		}
			// System.out.println("lifting");
		else if (!revup.hasElapsed(revtime)){
			shooterSubSys.autoRev();
			drivetrainSubSys.align();
			SmartDashboard.putString("auto state", "rev/align");

		}else if (!revup.hasElapsed(finaltime)){
			shooterSubSys.shoot();
			// System.out.println("reving");
			
			SmartDashboard.putString("auto state", "shooting");
		} else {
			shooterSubSys.stop();
			angleSubSys.descend();
			
			SmartDashboard.putString("auto state", "stopping");
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrainSubSys.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
