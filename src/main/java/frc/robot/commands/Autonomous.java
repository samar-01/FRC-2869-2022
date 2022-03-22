// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
	// boolean autoCheck(automodes a){
	// 	return (autoPickerEntry.equals(a));
	// }

	/** Creates a new Autonomous. */
	public Autonomous(double autorotate) {
		// if (autopicker.getSelected() == automodes.backupShoot){
		// 	addCommands(new DriveBackAndCalibClimb(), new LiftAlignShoot(), new AutoDown(RobotContainer.angleSubSys, RobotContainer.shooterSubSys));
		// }
		// else if (autopicker.getSelected() == automodes.backupOnly){
		// 	addCommands(new DriveBackAndCalibClimb());
		// }
		
		// if (autoCheck(automodes.backupOnly)){
		// 	addCommands(new DriveBackAndCalibClimb());
		// 	System.out.println("backup");
		// }
		
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		// addCommands(new DriveBackAndCalibClimb(), new LiftAlignShoot(), new DownRotate(autorotate), new FindBallShoot());
		addCommands(new DriveBackAndCalibClimb(), new LiftAlignShoot(), new DownRotate(autorotate));
		// addCommands(new DriveBackAndCalibClimb(), new LiftAlignShoot(), new AutoDown(RobotContainer.angleSubSys, RobotContainer.shooterSubSys));
		// addCommands(new DriveDistance(RobotContainer.drivetrainSubSys, -10));
	}
}
