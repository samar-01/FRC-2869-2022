// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fenderballtrack extends SequentialCommandGroup {
  /** Creates a new fenderballtrack. */
  public fenderballtrack() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new CloseHigh(), new AutoDown(), new Drive180(), new ResetGyro(), new PointIntake(), new DriveIntake(), new Drive180());
    // addCommands(new CloseHigh(), new AutoDown(), new ResetGyro(), new Drive180(), new PointIntakeDrive(), new DriveIntake(), new Drive180());
    addCommands(new CloseHigh(), new Down180(), new PointIntakeDrive(0.5), new DriveBackToTarget(), new Up180(), new CloseHigh(), new AutoDown());
  }
}
