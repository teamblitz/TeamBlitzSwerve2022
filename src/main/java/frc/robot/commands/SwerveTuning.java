package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveTuning extends CommandBase {
    private final DriveSubsystem driveSubsystem;

    private final ShuffleboardTab tab;

    private final GenericEntry angle;
    //    private final GenericEntry
    //    private final GenericEntry speed;

    public SwerveTuning(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);

        this.tab = Shuffleboard.getTab("DriveTuning");
        angle = tab.add("angle", 0).getEntry("double");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (DriverStation.isTest()) {
            driveSubsystem.setModuleStates(
                    new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getDouble(0))),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getDouble(0))),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getDouble(0))),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(angle.getDouble(0)))
                    },
                    true);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !DriverStation.isTest();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new Translation2d(0, 0), 0, false, true);
    }
}
