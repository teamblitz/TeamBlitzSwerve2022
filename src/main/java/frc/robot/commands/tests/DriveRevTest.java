package frc.robot.commands.tests;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveRevTest extends SequentialCommandGroup {
    public DriveRevTest(DriveSubsystem m_drive) {
        addCommands(
            new StartEndCommand(() -> m_drive.performDrive(.25, 0, false, false), ()-> m_drive.performDrive(0, 0, false, false), m_drive).withTimeout(10),
            new WaitCommand(5),
            new StartEndCommand(() -> m_drive.performDrive(.50, 0, false, false), ()-> m_drive.performDrive(0, 0, false, false), m_drive).withTimeout(10),
            new WaitCommand(5)
        );
    }
}
