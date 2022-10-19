/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.SaitekX52Joystick;

/**
* The Constants class provides a convenient place for teams to hold robot-wide
* numerical or boolean constants. This class should not be used for any other
* purpose. All constants should be declared globally (i.e. public static). Do
* not put anything functional in this class.
*
* <p>
* It is advised to statically import this class (or one of its inner classes)
* wherever the constants are needed, to reduce verbosity.
*/
public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontLeftDrive = 0;
        public static final int kFrontLeftSteer = 0;
        public static final int kFrontLeftSteerEncoder = 0;
        public static final int kFrontLeftSteerOffset = 0;

        public static final int kFrontRightDrive = 0;
        public static final int kFrontRightSteer = 0;
        public static final int kFrontRightSteerEncoder = 0;
        public static final int kFrontRightSteerOffset = 0;

        public static final int kBackLeftDrive = 0;
        public static final int kBackLeftSteer = 0;
        public static final int kBackLeftSteerEncoder = 0;
        public static final int kBackLeftSteerOffset = 0;

        public static final int kBackRightDrive = 0;
        public static final int kBackRightSteer = 0;
        public static final int kBackRightSteerEncoder = 0;
        public static final int kBackRightSteerOffset = 0;

        
        public static final int k = 4;
        public static final int kLeftSlavePort = 3;

        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.042545;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0; // To fix.

    }
    
    
    public static final class OIConstants {

        // Choose 1, not both.
        public static final boolean useXboxController = false;
        public static final boolean useSaitekController = true;
        
        public static final int kDriveControllerPort = 0; 
        public static final int kButtonBoxPort = 1;
        
        // Xbox buttons:
        
        public static final class XboxMappings {

            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final XboxController.Axis kOverdrive =  XboxController.Axis.kRightTrigger;
            
            public static final XboxController.Button kUpElevator = XboxController.Button.kY; // Should be the Y button
            public static final XboxController.Button kDownElevator = XboxController.Button.kA; // Should be the A button
            
            public static final XboxController.Button kIntake = XboxController.Button.kB; // Should be the B button
            public static final XboxController.Button kBallMover = XboxController.Button.kX; // Should be the X button
            public static final XboxController.Button kBallMoverReversed = XboxController.Button.kBack; // Back Button
            public static final XboxController.Button kShooter = XboxController.Button.kRightBumper;  // Should be right bumper  
            public static final XboxController.Button kShooterReversed = XboxController.Button.kStart; // Start button
            
            public static final XboxController.Button kSemiAutoBallSeek = XboxController.Button.kLeftBumper; // Auto Ball seek is on left bumper
            public static final XboxController.Axis kSemiAutoBallTarget = XboxController.Axis.kLeftTrigger; // Auto target is on left analog trigger
        }

        public static final class SaitekMappings {
            // We can do Button.kA.value to get the value of button A.
            // We could make the constants enums and do .value when binding
            public static final SaitekX52Joystick.Axis kThrotle = SaitekX52Joystick.Axis.kThrotle;
            
            public static final SaitekX52Joystick.Button kUpElevator = SaitekX52Joystick.Button.kT1; 
            public static final SaitekX52Joystick.Button kDownElevator = SaitekX52Joystick.Button.kT2;
            
            public static final SaitekX52Joystick.Button kIntake = SaitekX52Joystick.Button.kFire;
            public static final SaitekX52Joystick.Button kIntakeReversed = SaitekX52Joystick.Button.kHatDown;
            public static final SaitekX52Joystick.Button kBallMover = SaitekX52Joystick.Button.kUpperTrigger1;
            public static final SaitekX52Joystick.Button kBallMoverReversed = SaitekX52Joystick.Button.kHatDown;
            public static final SaitekX52Joystick.Button kShooter = SaitekX52Joystick.Button.kLowerTrigger;  
            public static final SaitekX52Joystick.Button kShooterReversed = SaitekX52Joystick.Button.kHatDown;
            
            public static final SaitekX52Joystick.Button kSemiAutoBallSeek = SaitekX52Joystick.Button.kA;
            public static final SaitekX52Joystick.Button kSemiAutoBallTarget = SaitekX52Joystick.Button.kB;
        }

        public static final class ButtonBoxMappings {
            public static final int kUpElevator = ButtonBox.Button.kL1.value;
            public static final int kDownElevator = ButtonBox.Button.kL2.value;
            
            public static final int kIntake = ButtonBox.Button.kX.value;
            public static final int kIntakeReversed = ButtonBox.Button.kL3.value;
            public static final int kBallMover = ButtonBox.Button.kY.value;
            public static final int kBallMoverReversed = ButtonBox.Button.kR3.value;
            public static final int kShooter = ButtonBox.Button.kR1.value; 
            public static final int kShooterReversed = ButtonBox.Button.kB.value;

            public static final int kSemiAutoBallSeek = ButtonBox.Button.kA.value;
            public static final int kSemiAutoBallTarget = ButtonBox.Button.kR2.value;
        }
    }
}