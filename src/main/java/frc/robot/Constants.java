/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
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

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSSwerveConstants chosenModule =
                COTSSwerveConstants.SDSMK4i(COTSSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Current Limits
        Smart current limit is */
        public static final int driveSmartCurrentLimit = 40; // TODO: Make sure these are acceptable values.
        public static final int driveSecondaryCurrentLimit = 65;

        public static final int angleSmartCurrentLimit = 25;
        public static final int angleSecondaryCurrentLimit = 40;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /**
         * Meters per Second
         */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
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


    public static double stickDeadband = 0.1;
}