package org.firstinspires.ftc.teamcode.Drive.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class DriveSubsystem extends SubsystemBase {
    //private Follower follower;

    private final MecanumDrive m_drive;

    RevIMU imu;

    GamepadEx driverOp = new GamepadEx(gamepad1);

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight, RevIMU imu) {

        m_drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
    public DriveSubsystem(HardwareMap hMap, final String frontLeftMotorName, String frontRightMotorName, String backLeftMotorName, String backRightMotorName, String ImuName) {
        this(new MotorEx(hMap, frontLeftMotorName), new MotorEx(hMap, frontRightMotorName), new MotorEx(hMap, backLeftMotorName), new MotorEx(hMap, backRightMotorName), new RevIMU(hMap, ImuName));
    }
    public void drive() {
//        follower.startTeleopDrive();
//        follower.setTeleOpMovementVectors(-driverOp.getLeftY(), -driverOp.getLeftX(), -driverOp.getRightX());
//        follower.update();

        m_drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                false
        );
    }
}