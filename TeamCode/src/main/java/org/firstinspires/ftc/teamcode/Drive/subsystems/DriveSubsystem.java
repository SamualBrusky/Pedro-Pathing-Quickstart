package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class DriveSubsystem extends SubsystemBase {

    public DriveSubsystem(HardwareMap hardwareMap) {}

    public void PedroDrive(Follower follower, double x, double y, double z) {
        follower.setTeleOpMovementVectors(x, y, z, true);
        follower.update();
    }
}