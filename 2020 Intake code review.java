    switch (_ballStatus) {

    case 0:

      // neutral position, not doing anything, robot starts in this configuration

      _intakeWheel.set(0);
      _rearIntakeBelt.set(0);
      _frontIntakeBelt.set(0);
      //_intakePneumatics.set(false);
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
      _intakeWheel.set(-0.9);
      _frontIntakeBelt.set(0.9);

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
      //_intakePneumatics.set(false);
      System.out.println("Step 2 Encoder Front: " + _frontIntakeBelt.getSelectedSensorPosition());
      _intakeWheel.set(0);

      if (_frontIntakeBelt.getSelectedSensorPosition() < CPBL) {

        _frontIntakeBelt.set(0.4);
        _rearIntakeBelt.set(-0.21);

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
        _rearIntakeBelt.set(-0.21);

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
        _rearIntakeBelt.set(0.21);
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
      //_intakePneumatics.set(false);
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
