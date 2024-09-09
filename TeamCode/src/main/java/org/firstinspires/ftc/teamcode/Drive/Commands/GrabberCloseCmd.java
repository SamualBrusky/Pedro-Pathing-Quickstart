package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.GrabberSubsystem;

import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input
 * (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class GrabberCloseCmd extends CommandBase {

    private final GrabberSubsystem m_Grabber;

    public GrabberCloseCmd(GrabberSubsystem subsystem) {
        m_Grabber = subsystem;
        addRequirements(m_Grabber);
    }

    @Override
    public void execute() {

        m_Grabber.rotationToPosition(GrabberSubsystem.GrabberPosition.CLOSED_POSITION);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}