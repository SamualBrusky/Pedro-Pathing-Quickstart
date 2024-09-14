package org.firstinspires.ftc.teamcode.Drive.subsystems;

import static com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt.getAngle;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class DriveSubsystem extends SubsystemBase {

    private final MotorEx m_FrontLeft;
    private final MotorEx m_FrontRight;
    private final MotorEx m_BackLeft;
    private final MotorEx m_BackRight;

    private final RevIMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        m_FrontLeft = hardwareMap.get(MotorEx.class, "FrontLeft");
        m_FrontRight = hardwareMap.get(MotorEx.class, "FrontRight");
        m_BackLeft = hardwareMap.get(MotorEx.class, "BackLeft");
        m_BackRight = hardwareMap.get(MotorEx.class, "BackRight");

        imu = hardwareMap.get(RevIMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        m_FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void PedroDrive(Follower follower, double x, double y, double z, RevIMU imu) {
        follower.setTeleOpMovementVectors(x, y, z, false);
        follower.update();
    }

    public double getHeading() {
        return imu.getHeading();
    }

    public void resetAngle() {
        imu.reset();
    }

    public void StopDrive(Follower follower){
        follower.setTeleOpMovementVectors(0,0,0, false);
    }
}
