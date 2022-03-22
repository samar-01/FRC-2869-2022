// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.Angle;
import frc.robot.commands.AutoPointGoal;
import frc.robot.commands.Autonomous;
import frc.robot.commands.Climber;
import frc.robot.commands.Drive180;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Drivetrain;
import frc.robot.commands.PreRev;
import frc.robot.commands.Shooter;
import frc.robot.commands.DriveReset;
import frc.robot.commands.DriveStop;
import frc.robot.subsystems.DrivetrainSubSys;
import frc.robot.subsystems.LimelightSubSys;
import frc.robot.subsystems.ShooterSubSys;
import frc.robot.subsystems.AngleSubSys;
import frc.robot.subsystems.ClimberSubSys;

import static frc.robot.Constants.*;

import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	double autorotate = 120;
	ShuffleboardTab auto = Shuffleboard.getTab("Auto");
	ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");

	public static SendableChooser autopicker = new SendableChooser();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		initFlash();
		AngleSubSys.init();
		RobotContainer.climberSubSys.init();
		networkInit();
		offLime();

		ShuffleboardTab auto = Shuffleboard.getTab("Auto");
		// autorotateEntry = auto.add("autorotate", autorotate).withPosition(0, 0).withSize(1, 1).getEntry();
		angleEntryA = auto.add("Angle", -34).withPosition(0, 1).withSize(1, 1).getEntry();
		batVoltageEntryA = auto.add("Voltage", 0).withPosition(1, 1).withSize(1, 1).getEntry();
		timeA = auto.add("time", 0).withPosition(2, 1).getEntry();
		autopicker.setDefaultOption("backup shoot", automodes.backupShoot);
		autopicker.addOption("backup only", automodes.backupOnly);
		autopicker.addOption("2 ball right", automodes.backupShootTurnRight);
		// auto.add("autopicker", autopicker).withPosition(0, 0).withSize(3, 1);
		autoPickerEntry = auto.add("autoselector", 0).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(3, 1).getEntry();

		ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
		distanceEntry = teleop.add("Distance", 0.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 7)).withPosition(0, 0).withSize(3, 1).getEntry();
		velconstantEntry = teleop.addPersistent("velconstant", 7.77).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 6, "max", 10)).withPosition(0,1).withSize(3, 1).getEntry();
		ballinEntry = teleop.add("BallIn", false).withPosition(0, 2).withSize(1, 1).getEntry();
		revedEntry = teleop.add("Reved", false).withPosition(1, 2).withSize(1, 1).getEntry();
		angleEntryT = teleop.add("Angle", -34).withPosition(2, 2).getEntry();
		leftFalEntry = teleop.add("lfalc", 0).withPosition(0, 3).getEntry();
		rightFalEntry = teleop.add("rfalc", 0).withPosition(1, 3).getEntry();
		tarSpeedEntry = teleop.add("target", 0).withPosition(2, 3).getEntry();
		batVoltageEntryT = teleop.add("battery",0).withPosition(0, 4).getEntry();
		timeT = teleop.add("time", 0).withPosition(1, 4).getEntry();
		// tarspeedthing = teleop.add("tarspeedthing", 7000).withPosition(4, 4).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 7000, "max", 9000)).withSize(3, 1).getEntry();
		Shuffleboard.selectTab("Auto");

		batVoltageEntry = batVoltageEntryA;
		angleEntry = angleEntryA;
		time = timeA;
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		batVoltageEntry.setNumber(RobotController.getBatteryVoltage());
		time.setNumber(Timer.getMatchTime());
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		DrivetrainSubSys.stopDrive();
		offFlash();
		networkInit();
		offLime();
	}

	@Override
	public void disabledPeriodic() {

	}

	// void addData(String tab, String label, Sendable sendable){
	// 	try {
	// 		Shuffleboard.getTab(tab).add(label, sendable);
	// 	} catch (IllegalArgumentException e){
			
	// 	}
	// }

	// Timer autodrive = new Timer();
	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		batVoltageEntry = batVoltageEntryA;
		angleEntry = angleEntryA;
		time = timeA;
		photonResetPipe();
		Shuffleboard.selectTab("Auto");
		autorotate = SmartDashboard.getNumber("autorotate", autorotate);
		RobotContainer.drivetrainSubSys.autoInit();

		m_autonomousCommand = new Autonomous(autorotate);

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			System.out.println("auto scheduled");
			m_autonomousCommand.schedule();
		}
		
		initLime();
		// autodrive.reset();
		// autodrive.start();
		// AngleSubSys.init();
	}
	double backup = 5;
	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// autoSchedule(RobotContainer.autonomous);
		// if (!autodrive.hasElapsed(backup)){
			// 	DrivetrainSubSys.drv(-0.40, 0);
			// } else {
		// 	DrivetrainSubSys.stopDrive();
		// }
		
		// if (autodrive.hasElapsed(backup+0.2)){
		// 	AngleSubSys.lift();
		// }
		// if (autodrive.hasElapsed(backup+0.2) && AngleSubSys.isLifted()){
		// 	ShooterSubSys.revup();
		// }
	}

	@Override
	public void teleopInit() {
		batVoltageEntry = batVoltageEntryT;
		angleEntry = angleEntryT;
		time = timeT;
		Shuffleboard.selectTab("Teleop");
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		initLime();

		/* photon vision temp comment
		autoDriveButton.whenPressed(new DriveAuto(new DrivetrainSubSys()));
		resetDriveButton.whenPressed(new DriveReset(new DrivetrainSubSys()));
		stopDriveButton.whenPressed(new DriveStop(new DrivetrainSubSys()));
		
		driveDriveButton.whenPressed(new DriveDistance(new DrivetrainSubSys(),20));
		*/
		
		// spinDriveButton.whenPressed(new Drive180(new DrivetrainSubSys()));
		
		// offFlash();
		onFlash();
		onLime();
		// pointDriveButton.whenPressed(new AutoPoint(new DrivetrainSubSys()));

	}

	public void initLime(){
		networkInit();
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
	}
	
	
	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		// System.out.println(velconstantEntry.getNumber(0));
		// System.out.println(ShooterSubSys.isIntakeEmpty());

		if (opxbox.getRightStickButtonPressed()){
			if (getFlash() > 0){
				offFlash();
			} else {
				onFlash();
			}
		}

		if (!RobotContainer.pointIntake.isScheduled()){
			if (xbox.getPOV() == 90 && !RobotContainer.pointIntake.isScheduled()){
				autoSchedule(RobotContainer.pointIntake);
			} else {
				autoSchedule(RobotContainer.drivetrain);
				autoSchedule(RobotContainer.shooter);
			}
		} else {
			RobotContainer.drivetrain.cancel();
			RobotContainer.shooter.cancel();
		}

		if (xbox.getPOV() == 270){
			RobotContainer.pointIntake.cancel();
		}

		autoSchedule(RobotContainer.climber);
		if (opxbox.getAButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoLift);
		} else if (opxbox.getBButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoDown);
		} else if (opxbox.getYButton()){
			// RobotContainer.shooterSubSys.rev();
			autoSchedule(RobotContainer.autoMid);
		} else if (!RobotContainer.pointIntake.isScheduled() && !RobotContainer.autoLift.isScheduled() && !RobotContainer.autoDown.isScheduled() && !RobotContainer.autoMid.isScheduled()){
			autoSchedule(RobotContainer.shooter);
			autoSchedule(RobotContainer.angle);
		}
		
		LimelightSubSys.getDistance();
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
