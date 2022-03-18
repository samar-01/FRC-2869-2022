// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Angle;
import frc.robot.commands.Drive180;
import frc.robot.commands.AutoPoint;
import frc.robot.commands.Autonomous;
import frc.robot.commands.BallTrack;
import frc.robot.commands.CalibClimb;
import frc.robot.commands.Climber;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveReset;
import frc.robot.commands.Drivetrain;
import frc.robot.commands.AutoLift;
import frc.robot.commands.Shooter;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.ClimberSubSys;
// import frc.robot.subsystems.AutomaticSubSys;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.ShooterSubSys;
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
	public static final DriveAuto driveauto = new DriveAuto(drivetrainSubSys);
	public static final DriveReset drivereset = new DriveReset(drivetrainSubSys);
	public static final Drive180 drive180 = new Drive180(drivetrainSubSys);
	public static final BallTrack ballTrack = new BallTrack(drivetrainSubSys);
	public static final AutoPoint point = new AutoPoint(drivetrainSubSys);
	public static final ShooterSubSys shooterSubSys = new ShooterSubSys();
	public static final Shooter shooter = new Shooter(shooterSubSys);
	public static final AngleSubSys angleSubSys = new AngleSubSys();
	public static final Angle angle = new Angle(angleSubSys);
	public static final AutoLift autolift = new AutoLift(angleSubSys);
	public static final Autonomous autonomous = new Autonomous(drivetrainSubSys, angleSubSys, shooterSubSys);
	public static final ClimberSubSys climberSubSys= new ClimberSubSys();
	public static final Climber climber = new Climber(climberSubSys);
	public static final CalibClimb calibClimb = new CalibClimb(climberSubSys);
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
