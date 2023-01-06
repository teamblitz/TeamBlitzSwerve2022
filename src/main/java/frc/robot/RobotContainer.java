/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.SaitekX52Joystick;
import frc.robot.Constants.OIConstants.SaitekMappings;
import frc.robot.commands.SwerveTuning;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */

    private DriveSubsystem driveSubsystem;

    /* ***** --- Controllers --- ***** */
    private XboxController driveController;
    private SaitekX52Joystick otherDriveController;

    private SwerveTuning tuningCommand; 

    public RobotContainer() {
        configureSubsystems();

        // CameraServer.startAutomaticCapture(); // Ignore warning.

        configureButtonBindings();

        
        ShuffleboardTab tuningTab = Shuffleboard.getTab("DriveTuning");
        
        this.tuningCommand = new SwerveTuning(driveSubsystem);
        
        setDefaultCommands();
        // tuningTab.add("Tuning Command", tuningCommand);

        // tuningCommand.schedule();

        new Trigger(driveController::getAButton)
                .onTrue(new InstantCommand(() -> tuningCommand.nextAngle()));
    }

    private void setDefaultCommands() {
        // Set default command for drive
        // driveSubsystem.setDefaultCommand(
        //         new TeleopSwerve(
        //                 driveSubsystem,
        //                 () -> -driveController.getLeftY() * .2,
        //                 () -> -driveController.getLeftX() * .2,
        //                 () -> -driveController.getRightX() * .2,
        //                 () -> false));
        driveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        driveSubsystem,
                        () -> -otherDriveController.getRawAxis(SaitekX52Joystick.Axis.kYAxis.value) * .5,
                        () -> -otherDriveController.getRawAxis(SaitekX52Joystick.Axis.kXAxis.value) * .5,
                        () -> -otherDriveController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value) * .2,
                        () -> false));
        // driveSubsystem.setDefaultCommand(tuningCommand);
    }

    private void configureSubsystems() {
        driveSubsystem =
                new DriveSubsystem(
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS),
                        new GyroIONavx());
        driveController = new XboxController(0);
        otherDriveController = new SaitekX52Joystick(1);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new Trigger(driveController::getYButton).onTrue(Commands.runOnce(driveSubsystem::zeroGyro));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return null;
    }
}
