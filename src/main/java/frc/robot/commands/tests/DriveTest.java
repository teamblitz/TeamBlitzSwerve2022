package frc.robot.commands.tests;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTest extends SequentialCommandGroup {
    public DriveTest(DriveSubsystem m_drive) {
        addCommands(
            new RunCommand(() -> m_drive.performDrive(.25, 0, false, false), m_drive).withTimeout(2),
            new RunCommand(() -> m_drive.performDrive(-.25, 0, false, false), m_drive).withTimeout(2),
            new RunCommand(() -> m_drive.performDrive(0, .25, false, false), m_drive).withTimeout(2),
            new RunCommand(() -> m_drive.performDrive(0, -.25, false, false), m_drive).withTimeout(2)
        );
    }
}
