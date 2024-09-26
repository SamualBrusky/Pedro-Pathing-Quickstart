package org.firstinspires.ftc.teamcode.Drive.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drive.Commands.DriveCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.GrabberCloseCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.GrabberOpenCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftHighBucketPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftIntakePosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftLowBucketPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftStartPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftSubmersibleScoreCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftSubmersibleSetupCmd;
import org.firstinspires.ftc.teamcode.Drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Blue TeleOp")
public class TeleOp extends CommandOpMode {
    //drive subsystem
   // private MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private IMU m_imu;
    //GamePads
    public GamepadEx m_driverOp, m_engineerOp;
    public Follower follower;

//    private LiftHighBucketPosCmd m_LiftHighBucketPosCmd;
//    private LiftStartPosCmd m_LiftStartPosCmd;
      private LiftIntakePosCmd m_LiftintakePosCmd;
//    private GrabberOpenCmd m_GrabberOpenCmd;
//    private GrabberCloseCmd m_GrabberCloseCmd;
//    private LiftSubmersibleSetupCmd m_LiftSubmersibleSetupCmd;
//    private LiftLowBucketPosCmd m_LiftLowBucketPosCmd;
//    private LiftSubmersibleScoreCmd m_LiftSubmersibleScoreCmd;
      DriveCmd m_DriveCmd;

    public DriveSubsystem m_DriveSubsystem;

//    private GrabberSubsystem m_GrabberSubsystem;
//    LiftSubsystem m_LiftSubsystem;


//    public void whileWaitingToStart() {
//      CommandScheduler.getInstance().run();
//    }
    @Override
    public void initialize() {
        m_imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        // Chasis Motors
//        m_LiftSubsystem = new LiftSubsystem(hardwareMap);
        m_DriveSubsystem = new DriveSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);
//        //Intake
//        m_GrabberSubsystem = new GrabberSubsystem(hardwareMap);
        //Gamepads
        m_driverOp = new GamepadEx(gamepad1);
        m_engineerOp = new GamepadEx(gamepad2);

        //init commands
//        m_LiftStartPosCmd = new LiftStartPosCmd(m_LiftSubsystem);
//        m_LiftHighBucketPosCmd = new LiftHighBucketPosCmd(m_LiftSubsystem);
        //m_LiftintakePosCmd = new LiftIntakePosCmd(m_LiftSubsystem);
//        m_GrabberCloseCmd = new GrabberCloseCmd(m_GrabberSubsystem);
//        m_GrabberOpenCmd = new GrabberOpenCmd(m_GrabberSubsystem);
//        m_LiftSubmersibleScoreCmd = new LiftSubmersibleScoreCmd(m_LiftSubsystem);
//        m_LiftLowBucketPosCmd = new LiftLowBucketPosCmd(m_LiftSubsystem);
//        m_LiftSubmersibleSetupCmd = new LiftSubmersibleSetupCmd(m_LiftSubsystem);


       // m_engineerOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(m_LiftintakePosCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(m_GrabberOpenCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(m_GrabberCloseCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(m_LiftHighBucketPosCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(m_LiftLowBucketPosCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(m_LiftSubmersibleScoreCmd);
//        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(m_LiftSubmersibleSetupCmd);
//

//        follower.setTeleOpMovementVectors(m_driverOp.getLeftY(), m_driverOp.getLeftX(), m_driverOp.getRightX(), false);
//        follower.update();

//        waitForStart();
        m_DriveSubsystem.setDefaultCommand( new DriveCmd(
                m_DriveSubsystem,
                follower,
                m_driverOp.getLeftY(),
                m_driverOp.getLeftX(),
                m_driverOp.getRightX()
        ));
//        while (opModeIsActive()) {
//
//
//
////            m_DriveSubsystem.PedroDrive(m_driverOp.getLeftY(), m_driverOp.getLeftX(), m_driverOp.getRightX());
//        }





//        }
    }
}