package org.firstinspires.ftc.teamcode.Drive.subsystems;

import static com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt.getAngle;

import org.firstinspires.ftc.robotcore.external.Func;
import  org.firstinspires.ftc.robotcore.external.Telemetry;


import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class DriveSubsystem extends SubsystemBase {

    private Follower follower;
    private final DcMotorEx m_FrontLeft;
    private final DcMotorEx m_FrontRight;
    private final DcMotorEx m_BackLeft;
    private final DcMotorEx m_BackRight;

    private final IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        m_FrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        m_FrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        m_BackLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        m_BackRight = hardwareMap.get(DcMotorEx.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

//        m_BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        m_FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        m_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void PedroDrive(Follower follower, double x, double y, double z) {
        follower.setTeleOpMovementVectors(
                x,
                y,
                z,
                false);
        follower.update();
    }

//    public double getHeading() {
//        return imu.getHeading();
//    }

//    public void resetAngle() {
//        imu.reset();
//    }
}
