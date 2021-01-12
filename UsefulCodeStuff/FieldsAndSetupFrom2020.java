


// Motors:
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

// Solenoids:
  Solenoid _scissorLift = new Solenoid(11, 0);
  Solenoid _scissorLiftPart2 = new Solenoid(11, 1);
  Solenoid _intakePneumatics = new Solenoid(11, 2);
  Solenoid _wheelSpinnerPneu = new Solenoid(11, 3);

// DigitalInput, the laser sensors:
  DigitalInput _ballInputSensor = new DigitalInput(0);
  DigitalInput _ballOutputSensor = new DigitalInput(1);
  DigitalInput _stopWinchSensor = new DigitalInput(2);

// Joystick
  Joystick _joystickL = new Joystick(1);
  Joystick _joystickR = new Joystick(2);
  Joystick _joystickM = new Joystick(3);
