package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InternalBallDetectorSubsystem;



public class DriveStraightWithDelay extends CommandBase {

	private final DriveSubsystem driveSubsystem;
	private final InternalBallDetectorSubsystem internalBallDetectorSubsystem;
	
	private final long duration;
	private final long delay;
	private final double speed;
	
	private long startTime;
	private boolean driveBack; // Do we need to drive back? if our ball detector was active recently then this won't activate as we got a ball
	

	public DriveStraightWithDelay(final DriveSubsystem driveSubsystem, InternalBallDetectorSubsystem internalBallDetectorSubsystem, final long duration, final double voltage, final long delay)
	{
		this.driveSubsystem = driveSubsystem;
		this.internalBallDetectorSubsystem = internalBallDetectorSubsystem;
		this.delay = delay;
		this.speed = voltage;
		this.duration = duration;

		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		addRequirements(driveSubsystem);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("Starting Drive");
		startTime = System.currentTimeMillis();
		driveBack = internalBallDetectorSubsystem.lastSeen() < 7000; //If we haven't seen the ball via internal detector in 7 seconds we need to drive backwords as our bot didn't pick up the ball
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		final long Cur_Time = System.currentTimeMillis();
        if ((Cur_Time - startTime > delay) && driveBack) {
            driveSubsystem.tankDrive(speed, -speed);
        }
	}

    // Called when isFinished returns ture
	@Override
    public void end(boolean interrupted) {
        driveSubsystem.tankDrive(0, 0);
        System.out.println("Ending DriveStraitWith Delay");
    }

	// Make this return true when this Command no longer needs to run execute()
	@Override
    public boolean isFinished() {
		return System.currentTimeMillis() - startTime > (duration + delay);
	}
}
