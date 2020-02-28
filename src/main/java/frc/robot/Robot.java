/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
    private static OI m_oi;
    private static Joystick driver;
    private static WPI_VictorSPX driveFL;
    private static WPI_VictorSPX driveFR;
    private static WPI_VictorSPX driveRL;
    private static WPI_VictorSPX driveRR;
    private static WPI_VictorSPX climbMotor;
    private static WPI_VictorSPX climbFollower;
    private static DifferentialDrive driveTrain;
    private static CameraServer camServer = CameraServer.getInstance();
    private static MjpegServer mjpegServer1;

    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    double autoStartTime;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        m_oi = new OI();

        // initialize driver to port 0
        driver = new Joystick(0);

        // initialize drive motors
        driveFL = new WPI_VictorSPX(1);
        driveRL = new WPI_VictorSPX(2);
        driveFR = new WPI_VictorSPX(3);
        driveRR = new WPI_VictorSPX(4);
        driveRL.follow(driveFL);
        driveRR.follow(driveFR);

        // initialize drive train
        driveTrain = new DifferentialDrive(driveFL, driveFR);

        // initialize climb motors
        climbMotor = new WPI_VictorSPX(5);
        climbFollower = new WPI_VictorSPX(6);
        climbFollower.follow(climbMotor);
        // usb camera code
        // Creates UsbCamera and MjpegServer [1] and connects them
        UsbCamera usbCamera = camServer.startAutomaticCapture(0);
        mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
        mjpegServer1.setSource(usbCamera);
        // option 2
        // CameraServer.getInstance().startAutomaticCapture();

        m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
        // chooser.addOption("My Auto", new MyAutoCommand());
        SmartDashboard.putData("Auto mode", m_chooser);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This function is called once each time the robot enters Disabled mode. You
     * can use it to reset any subsystem information you want to clear when the
     * robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString code to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons to
     * the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        autoStartTime = Timer.getFPGATimestamp();
        m_autonomousCommand = m_chooser.getSelected();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
         * switch(autoSelected) { case "My Auto": autonomousCommand = new
         * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
         * ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.start();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // use this for commands if supported
        // Scheduler.getInstance().run();

        // run auto for 4 seconds
        double runTime = Timer.getFPGATimestamp() - autoStartTime;
        if (runTime < 4) {
            driveTrain.arcadeDrive(-0.5, 0.0);
        } else {
            driveTrain.arcadeDrive(0.0, 0.0);
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // drive
        driveTrain.arcadeDrive(-driver.getRawAxis(1), driver.getRawAxis(4));

        // climb
        double upTrig = driver.getRawAxis(3); // right trigger
        double downTrig = driver.getRawAxis(2); // left trigger
        if (upTrig > 0.2) {
            climbMotor.set(upTrig * 0.5);
        } else if (downTrig > 0.2) {
            climbMotor.set(-downTrig * 0.5);
        } else {
            climbMotor.set(0);
        }
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
