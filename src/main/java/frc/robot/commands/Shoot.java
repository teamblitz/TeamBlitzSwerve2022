package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallMoverSubsystem;
import frc.robot.subsystems.ShooterSubsystem;



public class Shoot extends CommandBase {
	
	private final ShooterSubsystem shooterSubsystem;
    private final BallMoverSubsystem ballMoverSubsystem;
	private final long duration;
    private final long warmupPeriod;

	private long startTime;

	public Shoot(final ShooterSubsystem shooterSubsystem, final BallMoverSubsystem ballMoverSubsystem, final long warmupPeriod, final long duration){
		this.shooterSubsystem = shooterSubsystem;
		this.ballMoverSubsystem = ballMoverSubsystem;
		this.warmupPeriod = warmupPeriod;
		this.duration = duration;
		
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(shooterSubsystem, ballMoverSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Shoot");
		startTime = System.currentTimeMillis();
        shooterSubsystem.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		final long Cur_Time = System.currentTimeMillis();
        if (Cur_Time - startTime > warmupPeriod) {
            ballMoverSubsystem.start();
        }
	
	}

    // Called when isFinished returns ture
	@Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        ballMoverSubsystem.stop();
        System.out.println("Ending Shoot");
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		return System.currentTimeMillis() - startTime > (duration);
	}
}
