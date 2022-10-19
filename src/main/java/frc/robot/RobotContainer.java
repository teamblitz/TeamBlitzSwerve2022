/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveStraightWithDelay;
import frc.robot.commands.SeekBall;
import frc.robot.commands.Shoot;
import frc.robot.commands.Target;
import frc.robot.commands.tests.DriveTest;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.ButtonBinder;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.SaitekX52Joystick;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  /* ***** --- Subsystems --- ***** */

  private DriveSubsystem m_drive;


  /* ***** --- Controllers --- ***** */


  public RobotContainer() {
    configureSubsystems();
    // buildCommands();
    // setDefaultCommands();
    CameraServer.startAutomaticCapture();
    
    configureButtonBindings();
  
  }

  // private void setDefaultCommands() {
  //   // Set defalut command for drive

  //   if (OIConstants.useXboxController) {
  //     m_drive.setDefaultCommand(
  //       new RunCommand(() -> m_drive
  //       // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
  //       .performDrive(
  //         filter.calculate(
  //           -m_xboxController.getLeftY() * (m_xboxController.getRawAxis(OIConstants.XboxMappings.kOverdrive.value) < 0.5 ? kDriveLowSpeed : kDriveFullSpeed)), 
  //         filterRotation.calculate(
  //           m_xboxController.getRightX() * (m_xboxController.getRawAxis(OIConstants.XboxMappings.kOverdrive.value) < 0.5 ? kTurnLowSpeed : kTurnFullSpeed)),
  //         m_xboxController.getRawButton(OIConstants.XboxMappings.kSemiAutoBallSeek.value), //Turns on semiautonomous ball acquire
  //         m_xboxController.getRawAxis(OIConstants.XboxMappings.kSemiAutoBallTarget.value) > 0.5), //Turns on semiautonomous targeter on Left Trigger
  //         m_drive).withName("DriveDefalutCommand"));
  //   } else if (OIConstants.useSaitekController) {
      
  //     m_drive.setDefaultCommand(
  //       new RunCommand(() -> m_drive
  //       // To remove slew rate limiter remove the filter.calculate(), and filterRotation.calculate()
  //       .performDrive(
  //         filter.calculate(
  //           // -m_saitekController.getY() * (((-m_saitekController.getRawAxis(SaitekX52Joystick.Axis.kThrotle.value)+1)/2) * kDriveMultiplyer + kDriveMinSpeed)
  //           -m_saitekController.getY() * (m_saitekController.getRawButton(SaitekX52Joystick.Button.kModeBlue.value) ? 1.0 : m_saitekController.getRawButton(SaitekX52Joystick.Button.kModeRed.value) ? .5 : .75)
  //           ),
  //         filterRotation.calculate(
  //           kTurnLowSpeed * m_saitekController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value)),
  //         m_saitekController.getRawButton(OIConstants.SaitekMappings.kSemiAutoBallSeek.value) || m_buttonBoard.getRawButton(OIConstants.ButtonBoxMappings.kSemiAutoBallSeek), //Turns on semiautonomous ball acquire
  //         m_saitekController.getRawButton(OIConstants.SaitekMappings.kSemiAutoBallTarget.value) || m_buttonBoard.getRawButton(OIConstants.ButtonBoxMappings.kSemiAutoBallTarget)), //Turns on semiautonomous targeter on Left Trigger
  //         m_drive).withName("DriveDefalutCommand"));    }
  // }


  private void configureSubsystems() {
    m_drive = new DriveSubsystem();
  }


  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
      
    }

    public Command getAutonomousCommands() { // Autonomous code goes here
      return null;
    }

  }

