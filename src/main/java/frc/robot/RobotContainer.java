// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	// public static final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	// public static final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
	public static final DrivetrainSubSys drivetrainSubSys = new DrivetrainSubSys();
	public static final Drivetrain drivetrain = new Drivetrain(drivetrainSubSys);
	public static final DriveAuto driveauto = new DriveAuto();
	public static final DriveReset drivereset = new DriveReset();
	public static final Drive180 drive180 = new Drive180();
	public static final BallTrack ballTrack = new BallTrack();
	public static final AutoPointGoal point = new AutoPointGoal();
	public static final ShooterSubSys shooterSubSys = new ShooterSubSys();
	public static final Shooter shooter = new Shooter(shooterSubSys);
	public static final AngleSubSys angleSubSys = new AngleSubSys();
	public static final Angle angle = new Angle(angleSubSys);
	public static final AutoLift autoLift = new AutoLift();
	public static final AutoDown autoDown = new AutoDown();
	public static final AutoMid autoMid = new AutoMid();
	public static final CloseHigh closeHigh = new CloseHigh();
	// public static final Autonomous autonomous = new Autonomous(drivetrainSubSys, angleSubSys, shooterSubSys);
	// public static final DriveDistance driveBack = new DriveDistance(drivetrainSubSys, -10);
	// public static final Autonomous autonomous = new Autonomous();
	public static final PointIntakeDrive pointIntake = new PointIntakeDrive();
	public static final ClimberSubSys climberSubSys= new ClimberSubSys();
	public static final Climber climber = new Climber(climberSubSys);
	// public static final CalibClimb calibClimb = new CalibClimb(climberSubSys);
	// public static final AutoLift lift = new AutoLift(angleSubSys);
	// public static final AutomaticSubSys automaticSubSys = new AutomaticSubSys(angleSubSys, drivetrainSubSys, point, lift);
	// public static final Automatic automatic = new Automatic(automaticSubSys);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {}

	// Autonomous auto = new Autonomous();

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		// return m_autoCommand;
		return null;
	}
}
