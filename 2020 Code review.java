/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.*;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  // Devices
  WPI_TalonSRX _leftFront = new WPI_TalonSRX(1);
  WPI_TalonSRX _rghtFront = new WPI_TalonSRX(2);
  WPI_TalonSRX _leftFollower = new WPI_TalonSRX(3);
  WPI_TalonSRX _rghtFollower = new WPI_TalonSRX(4);
  WPI_TalonSRX _frontIntakeBelt = new WPI_TalonSRX(5);
  WPI_TalonSRX _wheelSpinner = new WPI_TalonSRX(6);
  WPI_TalonSRX _bottomShooter = new WPI_TalonSRX(7);
  WPI_TalonSRX _intakeWheel = new WPI_TalonSRX(8);
  WPI_TalonFX _winchLeft = new WPI_TalonFX(9);
  WPI_TalonFX _rearIntakeBelt = new WPI_TalonFX(10);

  Solenoid _scissorLift = new Solenoid(11, 0);
  Solenoid _scissorLiftPart2 = new Solenoid(11, 1);
  Solenoid _intakePneumatics = new Solenoid(11, 2);
  Solenoid _wheelSpinnerPneu = new Solenoid(11, 3);

  DigitalInput _ballInputSensor = new DigitalInput(0);
  DigitalInput _ballOutputSensor = new DigitalInput(1);
  DigitalInput _stopWinchSensor = new DigitalInput(2);

  Joystick _joystickL = new Joystick(1);
  Joystick _joystickR = new Joystick(2);
  Joystick _joystickM = new Joystick(3);

  // SHUFFLEBOARD STUFFS

  ShuffleboardTab tab = Shuffleboard.getTab("3489 2020");

  SendableChooser<String> autoChooser;

  NetworkTableEntry cameraSele;

  // Joystick Button Assignments

  // MANUIPULATOR JOYSTICK
  int _buttonCase1Swap = 1;
  int _buttonColorWheel = 4;
  int _buttonBLEH = 5;
  int _buttonReverseIndexPart1 = 11;
  int _buttonReverseIndexPart2 = 7;
  int _buttonScissorLiftRight = 8;
  int _buttonScissorLiftLeft = 6;
  // int _buttonUnStucc = 9;
  int _buttonRI = 10;

  // DRIVER JOYSTICK LEFT
  int _buttonToggleDirection = 7;

  // DRIVER JOYSTICK RIGHT
  // int _buttonToggleDirection = 2; because exists on both joysticks

  // Drive Systems
  DifferentialDrive _diffDrive = new DifferentialDrive(_rghtFront, _leftFront);
  DifferentialDrive _diffDriveBWD = new DifferentialDrive(_leftFront, _rghtFront);

  // Variables
  double autoStartTime = 0;

  double step1Time = 3; // How long you want to be on step 1 minus how long the sum of previous steps
                        // took, changes if future math is done.
  double step2Time = 5; // How long you want to be on step 2 minus how long the sum of previous steps
                        // took, changes if future math is done.

  double autoStopTimeStep1 = 0;
  double autoStopTimeStep2 = 0;

  int aStep = 1; // Holds which step in auto you are on

  int _camFeed = 1; // holds which state the camera is in (1 is intake, 0 is outtake)
  int _ballStatus = 0;

  int _ballCount = 3;

  NetworkTableEntry _intBallCount;

  boolean fwdOn = true;
  boolean bwdOn = false;
  boolean reverseIndex = true;

  boolean goBWD = true;

  // double CPI = 216.7;
  double CPBL = 12000;
  // double CPBL = 6318.2018;
  // ^^^^ old bot number ^^^^
  double CPI = 215.608333;
  // double CPI = 1836.1811887807;
  // ^^^^ new bot number ^^^^

  int FWD = 0;
  boolean toggleBlocker = false;
  boolean toggleBlockerBlocker = false;
  double whenToggleBlockerCanBeSetToFalse = 0;
  int POneOrNOne = 1;

  boolean notToggledRecently = true;

  double pDriveSpeed = 0.65;
  double nDriveSpeed = -0.5;
  double pTurnSpeed = 0.75;
  double nTurnSpeed = -0.75;
  double pIntakeSpeed = 0.45;
  double nIntakeSpeed = -0.85;

  boolean autoDone = false;

  int _delayTime = 3;

  boolean latch = true;
  double stopJustStop = 0;

  boolean hasRun = false;

  boolean riLatch = true;
  double riLatchResetTime = 0;

  NetworkTableEntry cameraSelection;
  UsbCamera frontCam;
  UsbCamera rearCam;
  VideoSink server;

  int multiuseInt = 0;

  // boolean notConveyorBlocker = false;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    multiuseInt = 0;

    _leftFront.setSelectedSensorPosition(0);
    _rghtFront.setSelectedSensorPosition(0);

    latch = true;
    stopJustStop = 0;

    riLatch = true;
    riLatchResetTime = 0;

    aStep = 1;


    // .setFPS(20)
    // .setResolution(100, 100);

    autoChooser = new SendableChooser<String>();
    autoChooser.setDefaultOption("1,SCORE, PICK 2 OUR TRENCH", "1");
    autoChooser.addOption("2,PICK 2 CENTER RIGHT, SCORE", "2");
    autoChooser.addOption("3,PICK 2 OUR TRENCH, SCORE", "3");
    autoChooser.addOption("4,PICK 2 CNTR LEFT, STAY LEFT, SCORE", "4");
    autoChooser.addOption("5,PICK 2 OPP TRENCH, SCORE", "5");
    autoChooser.addOption("6,PICK 2 CNTR LEFT, SCORE", "6");
    autoChooser.addOption("7,SCORE, BACK UP", "7");
    autoChooser.addOption("8,PICK 3 OUR TRENCH", "8");
    autoChooser.addOption("9, BASICALLY 7 BUT WITH A 3 SECOND DELAY", "9");
    autoChooser.addOption("DO WHILE TEST", "10");

    Shuffleboard.getTab("3489 2020").add(autoChooser).withSize(2, 1);

    _intBallCount = tab.add("Ball Count", 3).withPosition(7, 0).withSize(2, 2).getEntry();

    // if(frontCam.isConnected() && rearCam.isConnected())
    {

      rearCam = CameraServer.getInstance().startAutomaticCapture(0);
      frontCam = CameraServer.getInstance().startAutomaticCapture(1);
      server = CameraServer.getInstance().getServer();

      frontCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      rearCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

      // cameraSelection =
      // NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

      // frontCam.setResolution(100, 100);
      // rearCam.setResolution(100, 100);

      // Commented out because is believed to stop the camera swap feature
      // Greyson Parr, 2/22/20 9:26AM

    }

    // else
    {

      // frontCam=CameraServer.getInstance().startAutomaticCapture(0);

    }

    // camFeed = CameraServer.getInstance().addSwitchedCamera("_cameraFeed");
    // camFeed.setSource(frontCam);
    // camFeed.setFPS(30);
    // camFeed.setResolution(100, 100);
    // also commented out because believed to break the camera swap, Greyson Parr,
    // 2/22/20 10:04AM
    // rearCamFeed = CameraServer.getInstance().startAutomaticCapture();

    // Shuffleboard.getTab("3489
    // 2020").add(camFeed.getSource()).withWidget(BuiltInWidgets.kCameraStream).withSize(4,
    // 4)
    // .withPosition(2, 0);
    // believed to only display the initial source of the camera feed, Greyson Parr
    // 2/22/20 10:05AM

    Shuffleboard.getTab("3489 2020").add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withSize(4, 4)
        .withPosition(2, 0);

    _rghtFollower.follow(_rghtFront);
    _leftFollower.follow(_leftFront);

    fwdOn = true;
    bwdOn = false;

    _diffDrive.setSafetyEnabled(false);
    _diffDriveBWD.setSafetyEnabled(false);
    _intakeWheel.setSafetyEnabled(false);

    _leftFront.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousInit() {

    _leftFront.setSelectedSensorPosition(0);
    _rghtFront.setSelectedSensorPosition(0);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

      autoStartTime = Timer.getFPGATimestamp();

      _ballCount = 3;

      autoStopTimeStep1 = (step1Time + autoStartTime);
      autoStopTimeStep2 = (step2Time + autoStartTime);

      _leftFront.setSelectedSensorPosition(0);
      _rghtFront.setSelectedSensorPosition(0);

      aStep = 1;
      latch = true;
      stopJustStop = 0;

      _ballCount = 3;

      _diffDrive.setSafetyEnabled(false); // Disables saftey for main drive diff drive
      // _intakeWheel.setSafetyEnabled(false); //Disables saftey for intake wheels
      // diff drive

    }
  }

  @Override
  public void autonomousPeriodic() {

    if (_ballInputSensor.get() == true) {

      _ballCount++;

    }

    if (_ballOutputSensor.get() == true) {

      _ballCount--;

    }

    String aAuto = autoChooser.getSelected(); // Holds what auto you are doing

    switch (aAuto) {
    case "1":
      auto1();
      System.out.println("Auto 1");
      break;

    case "2":
      auto2();
      System.out.println("Auto 2");
      break;

    case "3":
      auto3();
      System.out.println("Auto 3");
      break;

    case "4":
      auto4();
      System.out.println("Auto 4");
      break;

    case "5":
      auto5();
      System.out.println("Auto 5");
      break;

    case "6":
      auto6();
      System.out.println("Auto 6");
      break;

    case "7":
      auto7();
      System.out.println("Auto 7");
      break;

    case "8":
      auto8();
      System.out.println("Auto 8");
      break;

    case "9":
      auto9();
      System.out.println("Auto 9");
      break;

    case "10":
      testTurn();
      System.out.println("Test Turn Distance");
      break;
    }

    // Put emergency 1 second stop if encoder is still 0

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

    _ballStatus = 0;
    _ballCount = 0;

  }

  @Override
  public void teleopPeriodic() {

    if (_joystickM.getRawButton(_buttonColorWheel)) {

      _wheelSpinner.set(0.4);
      _wheelSpinnerPneu.set(true);

      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);
      _intakeWheel.set(0);
      _intakePneumatics.set(false);

    } else {

      _wheelSpinner.set(0);
      _wheelSpinnerPneu.set(false);

    }

    if (_joystickM.getRawButton(_buttonRI)) {
      if (riLatch) {
        riLatchResetTime = Timer.getFPGATimestamp() + 1;
        riLatch = false;
        System.out.println("Latch - RI Running?");
        multiuseInt = 1;
        _ballCount--;
      } // No work?
      System.out.println("Button 10 is pressed");
      ;
    }
    if ((Timer.getFPGATimestamp() >= riLatchResetTime) && !riLatch) {
      riLatch = true;
      System.out.println("Reset Latch?");
      riLatchResetTime = 0;
    }
    if (multiuseInt == 1) {
      ReverseIndex();
    }

    /*
     * if(_joystickL.getRawButton(_buttonIntakeWheelIn)) { _topShooter.set(-0.5);
     * _bottomShooter.set(-0.5);
     *
     * }
     *
     * if(_joystickL.getRawButton(_buttonIntakeWheelOut)) { _topShooter.set(0.5);
     * _bottomShooter.set(0.5);
     *
     * }
     *
     * if(_joystickL.getRawButton(_buttonIntakeBar)) { _intakeShooter.set(0.5);
     *
     * }
     *
     * else { _intakeShooter.set(0); }
     */
    // Drive

    // S_diffDrive.tankDrive(_joystickL.getY(), _joystickR.getY());

    // Reverse Code
    switch (FWD) {
    case 0:
      if (toggleBlocker == false
          && (_joystickL.getRawButton(_buttonToggleDirection) || _joystickR.getRawButton(_buttonToggleDirection))) {
        POneOrNOne = -1;
        toggleBlocker = true;
        toggleBlockerBlocker = true;
        whenToggleBlockerCanBeSetToFalse = Timer.getFPGATimestamp() + 1;
        FWD = 1;

        System.out.println("BWD");

      }
      // cameraSelection.setString(rearCam.getName());
      server.setSource(rearCam);
      _diffDrive.tankDrive(POneOrNOne * _joystickL.getY(), POneOrNOne * _joystickR.getY());
      // System.out.println("BWD mode");
      break;
    case 1:
      if (toggleBlocker == false
          && (_joystickL.getRawButton(_buttonToggleDirection) || _joystickR.getRawButton(_buttonToggleDirection))) {
        POneOrNOne = 1;
        toggleBlocker = true;
        toggleBlockerBlocker = true;
        whenToggleBlockerCanBeSetToFalse = Timer.getFPGATimestamp() + 1;
        FWD = 0;

        System.out.println("FWD");
        /*
         * if(frontCam.isConnected()) added to try to fix the camera swap, Greyson Parr
         * 2/22/20 10:30AM. It didnt work 10:38AM. { camFeed.setSource(frontCam); }
         */

        // Shuffleboard.getTab("3489
        // 2020").add(getSource(cameraSelection)).withWidget(BuiltInWidgets.kCameraStream).withSize(4,
        // 4)
        // .withPosition(2, 0);
      }
      // cameraSelection.setString(frontCam.getName());
      server.setSource(frontCam);

      _diffDrive.tankDrive(POneOrNOne * _joystickR.getY(), POneOrNOne * _joystickL.getY());
      // System.out.println("FWD mode");
      break;
    }
    if (toggleBlockerBlocker && (whenToggleBlockerCanBeSetToFalse < Timer.getFPGATimestamp())) {
      toggleBlocker = false;
      whenToggleBlockerCanBeSetToFalse = 0;
      toggleBlockerBlocker = false;
    }

    /*
     * if (!goBWD) {
     *
     * _diffDrive.tankDrive(_joystickL.getY(), _joystickR.getY());
     * System.out.println("Now in FWD");
     *
     * } else {
     *
     * _diffDriveBWD.tankDrive(-1 * _joystickR.getY(), -1 * _joystickL.getY());
     * System.out.println("Now in BWD"); }
     */
    // Winch Code
    if ((_joystickL.getPOV() == 180) && (_joystickR.getPOV() == 180)) {

      // _climbingWinch.tankDrive(0.85, 0.85);
      if (_stopWinchSensor.get()) {
        _winchLeft.set(0.5);
      }

      // _scissorLift.set(false);

    } else if ((_joystickL.getPOV() == 0) && (_joystickR.getPOV() == 0)) {
      _winchLeft.set(-0.3);
    } else {

      // _climbingWinch.tankDrive(0, 0);
      _winchLeft.set(0);

    }

    if (_joystickM.getRawButton(_buttonScissorLiftLeft)) {
      _scissorLiftPart2.set(false);
      _scissorLift.set(true);
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0); //changed in R1.2, sheer stupidity, 2/28/20, Greyson Parr

    }

    else {
      _scissorLift.set(false);
      _scissorLiftPart2.set(true);
    }

    /*
     * if (notToggledRecently) { if (goBWD &&
     * (_joystickL.getRawButton(_buttonToggleDirection))){ goBWD = false;
     * notToggledRecently = false; System.out.println("Changing to FWD"); } if
     * (!goBWD && (_joystickL.getRawButton(_buttonToggleDirection))){ goBWD = true;
     * notToggledRecently = false; System.out.println("Changing to BWD"); }
     *
     *
     * /* if (fwdOn && (_joystickL.getRawButton(_buttonToggleDirection) ||
     * _joystickR.getRawButton(_buttonToggleDirection))) { goBWD = false; bwdOn =
     * true; fwdOn = false; } if (bwdOn &&
     * (_joystickL.getRawButton(_buttonToggleDirection) ||
     * _joystickR.getRawButton(_buttonToggleDirection))) { goBWD = true; bwdOn =
     * false; fwdOn = true; } }
     */

    switch (_ballStatus) {

    case 0:

      // neutral position, not doing anything, robot starts in this configuration

      _intakeWheel.set(0);
      _rearIntakeBelt.set(0);
      _frontIntakeBelt.set(0);
      _intakePneumatics.set(false);
      reverseIndex = true;
      _frontIntakeBelt.setSelectedSensorPosition(0);

      if (!_ballInputSensor.get()) {
        _frontIntakeBelt.set(0.4);
      }

      // Checks to see if button to swap to case 1 is clicked

      if (_joystickM.getRawButton(_buttonCase1Swap)) {

        _ballStatus = 1;

      }

      // reverses intake and fixes ball count in the event of a failed ball capture

      if (_joystickM.getRawButton(_buttonReverseIndexPart1) && _joystickM.getRawButton(_buttonReverseIndexPart2)) {

        _ballStatus = 4;

      }

      // System.out.println("Case 0 is selected");
      // System.out.println(_ballCount + "is the ball count");

      break;

    case 1:
      // Intake starts spinning, waits for front sensor to be triggered

      // _intakeShooter.set(0.3);
      _intakePneumatics.set(true);
      _intakeWheel.set(-0.45);
      _frontIntakeBelt.set(0.5);

      // sets encoder clicks back to 0

      _frontIntakeBelt.setSelectedSensorPosition(0);
      if (_intakeWheel.getStatorCurrent() > 3.5) {
        _intakeWheel.set(.5);
      }
      // Once front sensor is triggered, case is switched to 2
      // System.out.println("Ball in intake?: " + !_ballInputSensor.get());
      if (!_ballInputSensor.get()) {

        _ballCount++;
        // System.out.println("Ball Count=" + _ballCount);

        if (_ballCount == 4) {
          _ballStatus = 3;
          // System.out.println("Going to case 3");
          break;

        }
        if (_ballCount == 5) {
          _ballStatus = 5;
          break;
        } else {

          _ballStatus = 2;
          // System.out.println("Going to case 2");
          break;

        }

      }

      // System.out.println("Case 1 is selected");

      break;

    case 2:

      // Intake system stops, followed by belts moving forward one ball length
      // System.out.println("Now in case 2 ballcount =" + _ballCount);
      _intakePneumatics.set(false);
      System.out.println("Step 2 Encoder Front: " + _frontIntakeBelt.getSelectedSensorPosition());
      _intakeWheel.set(0);

      if (_frontIntakeBelt.getSelectedSensorPosition() < CPBL) {

        _frontIntakeBelt.set(0.4);
        _rearIntakeBelt.set(-0.2);

      }

      else {

        _frontIntakeBelt.set(0);
        _rearIntakeBelt.set(0);
        _ballStatus = 0;
      }

      // System.out.println("Case 2 is selected");

      break;

    case 3:
      // Intake system stops, followed by locking everything except the outtake of all
      // cells at once
      // System.out.println("Now in case 3 ballcount = " + _ballCount);
      if (_frontIntakeBelt.getSelectedSensorPosition() < CPBL) {

        _frontIntakeBelt.set(0.4);
        _rearIntakeBelt.set(-0.2);

      }

      else {

        _frontIntakeBelt.set(0);
        _rearIntakeBelt.set(0);
        _ballStatus = 0;
        // _frontIntakeBelt.setSelectedSensorPosition(0);

      }

      break;

    case 4:
      if (_frontIntakeBelt.getSelectedSensorPosition() < (-1 * CPBL)) {

        _frontIntakeBelt.set(-0.4);
        _rearIntakeBelt.set(0.2);
        _intakeWheel.set(1);
        // _rearIntakeBelt.set(-0.5);

        /*
         * if(reverseIndex == true) {
         *
         * _ballCount --; reverseIndex = false;
         *
         * }
         */

      }

      else {

        _ballStatus = 0;

      }
      break;
    case 5:
      _intakePneumatics.set(false);
      _intakeWheel.set(0);
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);

      break;

    }

    // BLEH Code
    if ((_joystickM.getY()) > 0.35 || (_joystickM.getY() < -0.35)) {
      _frontIntakeBelt.set(_joystickM.getY() * -1);
      _rearIntakeBelt.set(_joystickM.getY());
      _intakeWheel.set(_joystickM.getY());
      _ballCount = 0;
      _ballStatus = 0;
      // set to 70% when it worked
      // _intakeWheel.set(-0.35);
    }
    /*
     * if (_joystickM.getRawButton(_buttonUnStucc)) { double time = 0; boolean
     * latchTime = true; _intakePneumatics.set(true); if (latchTime) { time =
     * Timer.getFPGATimestamp() + 1; latchTime = false; }
     *
     * if (Timer.getFPGATimestamp() < time) {
     * if(_frontIntakeBelt.getSelectedSensorPosition() < (-1 * CPBL)) {
     *
     * _frontIntakeBelt.set(-0.5); _rearIntakeBelt.set(-0.5);
     *
     * if(reverseIndex == true) {
     *
     * _ballCount --; reverseIndex = false;
     *
     * }
     *
     * } }
     *
     * else { _intakePneumatics.set(false); time = 0; latchTime = true; }
     *
     * }
     */

    _intBallCount.setNumber(_ballCount);

    _diffDrive.setSafetyEnabled(false);
    _frontIntakeBelt.setSafetyEnabled(false);
    _rearIntakeBelt.setSafetyEnabled(false);
    _winchLeft.setSafetyEnabled(false);

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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    autoStopTimeStep1 = 0;
    autoStopTimeStep2 = 0;

    autoStartTime = 0;

    aStep = 1;

    _ballCount = 3;
    _frontIntakeBelt.setSelectedSensorPosition(0);
    _leftFront.setSelectedSensorPosition(0);

    System.out.println("3489 2020 Code v02.27.2020 R1.2");

  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void auto1() {
    switch (aStep) {
    case 1:
      Drive(84);
      System.out.println("A1S" + aStep);
      break;

    case 2:
      Delay(1.5);
      System.out.println("A1S" + aStep);
      break;

    case 3:
      UNZUCC(2);
      System.out.println("A1S" + aStep);
      break;

    case 4:
      DriveBWD(84);
      System.out.println("A1S" + aStep);
      break;

    case 5:
      TurnRight(16);
      System.out.println("A1S" + aStep);
      break;

    case 6:
      Drive(46);
      System.out.println("A1S" + aStep);
      break;

    case 7:
      TurnLeft(30);
      System.out.println("A1S" + aStep);
      break;

    case 8:
      DriveBWD(20);
      System.out.println("A1S" + aStep);
      break;

    case 9:
      // DriveWithIntake(60);
      // IfDetectIndex();
      System.out.println("A1S" + aStep);
      break;

    default:
      aStep = 0;
      break;
    }
  }

  public void auto2() {
    switch (aStep) {
    case 1:
      Drive(112.2);
      break;

    case 2:
      TurnLeft(5);
      break;

    case 3:
      DriveWithIntake(5);
      IfDetectIndex();
      break;

    case 4:
      NP1TurnRight(5);
      break;

    case 5:
      TurnRight(5);

    case 6:
      DriveWithIntake(3);
      IfDetectIndex();
      break;

    case 7:
      NP1TurnLeft(5);
      IfDetectIndex();
      break;

    case 8:
      DriveBWD(224.4);
      IfDetectIndex();
      break;

    case 9:
      TurnLeft(26.58);
      break;

    case 10:
      Drive(74.55);
      break;

    case 11:
      TurnRight(26.58);
      break;

    case 12:
      UNZUCC(5);
      break;

    case 13:
      Drive(112.2);
      break;

    default:
      aStep = 0;
      break;
    }
  }

  public void auto3() {
    switch (aStep) {
    case 1:
      // DriveBWD(85);
      aStep++;
      break;

    case 2:
      DriveWithIntake(73);
      IfDetectIndex();
      break;

    case 3:
      Drive(50);
      break;

    case 4:
      // TurnRight(26.58);
      break;

    case 5:
      // DriveBWD(60);
      break;

    case 6:
      // TurnLeft(26.58);
      break;

    case 7:
      // Drive(101);
      break;

    case 8:
      // UNZUCC(5);
      break;

    default:
      aStep = 0;
      break;
    }
  }

  public void auto4() {
    switch (aStep) {
    case 1:
      DriveBWD(162);
      System.out.println("A4S" + aStep);
      break;
    case 2:
      TurnRight(40);
      System.out.println("A4S" + aStep);
      break;
    case 3:
      DriveBWD(24);
      System.out.println("A4S" + aStep);
      break;
    case 4:
      TurnRight(3);
      System.out.println("A4S" + aStep);
      break;
    case 5:
      DriveWithIntake(40);
      System.out.println("A4S" + aStep);
      break;

    default:
      aStep = 0;
      break;
    }
  }

  public void auto5() {
    switch (aStep) {
    case 1:
      Drive(85);
      break;

    case 2:
      TurnLeft(5);
      break;

    case 3:
      DriveWithIntake(5);
      break;

    case 4:
      NP1TurnRight(5);
      break;

    case 5:
      TurnLeft(5);
      DriveWithIntake(5);
      break;

    case 6:
      NP2TurnLeft(5);
      break;

    case 7:
      DriveBWD(85);
      break;

    case 8:
      TurnLeft(26.58);
      break;

    case 9:
      Drive(16.413);
      break;

    case 10:
      TurnLeft(26.58);
      break;

    case 11:
      Drive(85);
      break;

    case 12:
      DriveWithIntake(15);
      break;

    default:
      aStep = 0;
    }
  }

  public void auto6() {
    // these numbers are rough estimates, test them
    switch (aStep) {
    case 1:
      Drive(80);
      break;

    case 2:
      TurnRight(26.58);
      break;

    case 3:
      DriveWithIntake(10);
      break;

    case 4:
      DriveBWD(10);
      break;

    case 5:
      TurnLeft(26.58);
      break;

    case 6:
      Drive(7);
      break;

    case 7:
      TurnRight(26.58);
      break;

    case 8:
      DriveWithIntake(7);
      break;

    case 9:
      DriveBWD(7);
      break;

    case 10:
      TurnLeft(26.58);
      break;

    case 11:
      DriveBWD(162);
      break;

    case 12:
      UNZUCC(2);
      break;

    case 13:
      Drive(80);
      break;

    default:
      aStep = 0;
      break;
    }
  }

  public void auto7() {
    System.out.println("ECV for left is" + _leftFront.getSelectedSensorPosition());
    System.out.println("ECV for right is" + _rghtFront.getSelectedSensorPosition());
    switch (aStep) {
    case 1:
      Drive(84);
      System.out.println("Auto 7 Step 1");
      break;

    case 2:
      Delay(1.5);
      System.out.println("Auto 7 Step 2 Current Time: " + Timer.getFPGATimestamp());
      System.out.println("Target Time: " + stopJustStop);
      break;

    case 3:
      UNZUCC(2);
      System.out.println("Auto 7 Step 3");
      break;

    case 4:
      DriveBWD(80);
      System.out.println("Auto 7 Step 4");
      break;

    default:
      aStep = 0;
      _diffDrive.tankDrive(0, 0);
      break;
    }
  }

  public void auto8()
  // numbers are rough estimates, please test and/or change them
  {
    switch (aStep) {
    case 1:
      Drive(85);
      break;

    case 2:
      DriveWithIntake(110);
      IfDetectIndex();
      break;

    default:
      aStep = 0;
      _diffDrive.tankDrive(0, 0);
      break;
    }
  }

  public void auto9() {
    // literally auto 7 with a delay, added in R1.1, 2/27/20
    switch (aStep) {
    case 1:
      Delay(_delayTime);
      System.out.println("Delaying");
      break;

    case 2:
      Drive(84);
      System.out.println("Auto 7 Step 1");
      break;

    case 3:
      Delay(1.5);
      System.out.println("Auto 7 Step 2 Current Time: " + Timer.getFPGATimestamp());
      System.out.println("Target Time: " + stopJustStop);
      break;

    case 4:
      UNZUCC(2);
      System.out.println("Auto 7 Step 3");
      break;

    case 5:
      DriveBWD(80);
      System.out.println("Auto 7 Step 4");
      break;

    default:
      aStep = 0;
      _diffDrive.tankDrive(0, 0);
      break;
    }
  }

  public void testTurn() {

    if (DriveDoWhile(20) && !hasRun) {
      hasRun = true;
    }
    _diffDrive.tankDrive(0, 0);
    System.out.println("Encoders: " + Math.abs(_leftFront.getSelectedSensorPosition()));
    System.out.println("Should stop at:  " + (CPI * 20) + "clicks");
  }

  public boolean DriveDoWhile(double desiredDistance) {
    do {
      _diffDrive.tankDrive(pDriveSpeed, pDriveSpeed);
    } while (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI));
    _leftFront.setSelectedSensorPosition(0);
    _diffDrive.tankDrive(0, 0);
    aStep++;
    return true;
  }

  public void Drive(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(pDriveSpeed, pDriveSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
    System.out.println("Encoder clicks: " + _leftFront.getSelectedSensorPosition());
  }

  public void DriveBWD(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(nDriveSpeed, nDriveSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void TurnRight(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(pTurnSpeed, nTurnSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void TurnLeft(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(nTurnSpeed, pTurnSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void DriveWithIntake(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(nDriveSpeed, nDriveSpeed);
      _frontIntakeBelt.set(0.4);
      _rearIntakeBelt.set(-0.2);
      _intakePneumatics.set(true);
      _intakeWheel.set(-0.35);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);
      _intakeWheel.set(0);
      _intakePneumatics.set(false);
    }
  }

  public void UNZUCC(double seconds) {
    if (latch) {
      stopJustStop = (Timer.getFPGATimestamp() + seconds);
      latch = false;
    }

    if (Timer.getFPGATimestamp() < stopJustStop) {
      _rearIntakeBelt.set(nIntakeSpeed);
    } else {
      _rearIntakeBelt.set(0);
      latch = true;
      aStep++;
      stopJustStop = 0;
    }
  }

  public void NP1TurnRight(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(pTurnSpeed, 0);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void NP2TurnRight(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(0, nTurnSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void NP1TurnLeft(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(0, pTurnSpeed);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void NP2TurnLeft(double desiredDistance) {
    if (Math.abs(_leftFront.getSelectedSensorPosition()) < (desiredDistance * CPI)) {
      _diffDrive.tankDrive(nTurnSpeed, 0);
    } else {
      _diffDrive.tankDrive(0, 0);
      _leftFront.setSelectedSensorPosition(0);
      aStep++;
    }
  }

  public void Delay(double desiredSeconds) {
    if (latch) {
      stopJustStop = Timer.getFPGATimestamp() + desiredSeconds;
      latch = false;
    }
    if (Timer.getFPGATimestamp() > stopJustStop) {
      aStep++;
      latch = true;
    }
  }

  public void Index() {
    if (Math.abs(_frontIntakeBelt.getSelectedSensorPosition()) < CPBL) {
      _frontIntakeBelt.set(0.5);
      _rearIntakeBelt.set(-0.5);
    }

    else {
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);
      _frontIntakeBelt.setSelectedSensorPosition(0);
      multiuseInt = 0;
    }
  }

  public void ReverseIndex() {
    if (Math.abs(_frontIntakeBelt.getSelectedSensorPosition()) < CPBL) {
      _frontIntakeBelt.set(-0.5);
      _rearIntakeBelt.set(0.5);
    }

    else {
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);
      _frontIntakeBelt.setSelectedSensorPosition(0);
      multiuseInt = 0;
    }
  }

  public void IfDetectIndex() {
    if (_ballInputSensor.get()) {
      _frontIntakeBelt.set(0.5);
      _rearIntakeBelt.set(-0.3);
    } else {
      _frontIntakeBelt.set(0);
      _rearIntakeBelt.set(0);
    }
  }

  /*
   * public boolean driveDoWhile(double _ballCount) { return true;
   *
   * }
   */
}

// sorta kinda almost good
