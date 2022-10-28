/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {

  /* ***** --- Subsystems --- ***** */

  private Swerve m_drive;

  private XboxController m_driveController;


  /* ***** --- Controllers --- ***** */


  public RobotContainer() {
    configureSubsystems();
    // buildCommands();
    // setDefaultCommands();
    CameraServer.startAutomaticCapture();
    
    configureButtonBindings();
  
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    // Set defalut command for drive
    m_drive.setDefaultCommand(new TeleopSwerve(
            m_drive,
            () -> m_driveController.getLeftY(),
            () -> m_driveController.getLeftX(),
            () -> m_driveController.getRightX(),
            () -> true
            ));

  }


  private void configureSubsystems() {
    m_drive = new Swerve();
    m_driveController = new XboxController(0);
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

