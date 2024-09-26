package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class DriveCmd extends CommandBase {

    Follower m_follower;
    private DoubleSupplier x, y, z;
    private DriveSubsystem driveSubsystem;

    public DriveCmd(DriveSubsystem driveSubsystem, Follower follower, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
        m_follower = follower;
        this.x = x;
        this.y = y;
        this.z = z;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_follower.startTeleopDrive();
    }
    @Override
    public void execute() {
        driveSubsystem.PedroDrive(m_follower, x.getAsDouble(), y.getAsDouble(), z.getAsDouble());
    }

}