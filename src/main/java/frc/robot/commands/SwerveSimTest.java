package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveSimTest extends CommandBase{
    private Swerve s_Swerve;    
    private ShuffleboardTab tab;
    private NetworkTableEntry translationEntry;
    private NetworkTableEntry strafeEntry;
    private NetworkTableEntry rotationEntry;

    public SwerveSimTest(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        tab = Shuffleboard.getTab("Swerve");
        translationEntry = tab.add("Translation", 0).getEntry();
        strafeEntry = tab.add("Strafe", 0).getEntry();
        rotationEntry = tab.add("Rotation", 0).getEntry();
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(translationEntry.getDouble(0), strafeEntry.getDouble(0)), rotationEntry.getDouble(0) , false, true);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
