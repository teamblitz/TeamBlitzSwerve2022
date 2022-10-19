// Backup autonomous code. unused.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BallMoverSubsystem;

@Deprecated(forRemoval = true) // Don't use.
public class AutonomousCommand extends CommandBase {
    
    DriveSubsystem driveSubsystem;
	long	startTime;
	double	voltage;

    ShooterSubsystem shooterSubsystem;
    
    BallMoverSubsystem ballMoverSubsystem;
    
    int     stage = 1;
    // double	voltageAccommodater	= .5;	// Because our robot is part of the alt-right (or at least leans to the right)
										// Tests: VA Value | Distance | Avg. Deviation | Trials (Deviation to the Right,
										// Negative is Left)
										// 50% 30' 6"/30'; 1"/~5' 10", 12", -1", -10", 19"

    public AutonomousCommand(final DriveSubsystem driveSubsystem, final double voltage, final ShooterSubsystem shooterSubsystem, final BallMoverSubsystem ballMoverSubsystem) {
        // Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(driveSubsystem);
		this.driveSubsystem = driveSubsystem;
		this.voltage = voltage;
		
        this.ballMoverSubsystem = ballMoverSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    // Called just before this Command runs the first time
	@Override
	public void initialize() {
		startTime = System.currentTimeMillis();
        stage = 1;
	}

    // Called repeatedly when this Command is scheduled to run
    @Override
	public void execute() {

        if (stage==4){
            shooterSubsystem.stop();
            ballMoverSubsystem.stop();
            System.out.println("Finished");
            return;
        }

		final long Cur_Time = System.currentTimeMillis();

        System.out.print("Stage: ");
        System.out.println(stage);
        if (stage==1){
            //Insert shooter code
            if (Cur_Time - startTime < 1000) {
                System.out.print("S1 First condition");
                shooterSubsystem.start();
            }
            // else if ((Cur_Time - startTime < (shooter_duration + shooter_delay))) {
            //     shooterSubsystem.start();
            //     System.out.print("S1 Second condition");  // This makes it work; don't remove; don't ask
            // }
            else {
                shooterSubsystem.stop();
                System.out.println("S1 Stop condition. Entering stage 2");
                stage = 2;
            }
        }
        else if (stage==2){
            if (Cur_Time - startTime < 5000) {
                System.out.print("S2 First condition");
                shooterSubsystem.start();
                ballMoverSubsystem.start();
            }
            else {
                shooterSubsystem.stop();
                ballMoverSubsystem.stop();
                System.out.println("S2 Stop condition. Entering stage 3");
                stage = 3;
            }
        }
        else if (stage==3){
            //Insert drive code.
            System.out.println("stage 3");
            if (Cur_Time - startTime < 6000) {
                System.out.print("S3 First condition");
                driveSubsystem.tankDrive(voltage, -voltage);
            }
            // else if ((Cur_Time - startTime - shooter_duration < (duration + delay))) {
            //     driveSubsystem.tankDrive(voltage, -voltage);
            //     System.out.println("S3 Second condition");  // This makes it work; don't remove; don't ask
             else {
                driveSubsystem.tankDrive(0, 0);
                System.out.println("S3 Stop condition. Entering Stage 4");
                stage = 4;
            }
        }

	// @Override
	// public boolean isFinished() {
	// 	return System.currentTimeMillis() - startTime > (duration + shooter_duration + delay);
    // }
    }
}