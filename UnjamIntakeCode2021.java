private void tryToggleIntake()
    {
        if (robotHandler.inputHandler.shouldToggleIntake())
        {
            robotHandler.stateHandler.toggleIntake();
            intakeSolenoid.toggle();

            intakeState = IntakeState.IntakeDeactivated; //This is one of the problems, and the method call placement
        }
        if (isIntakeExtended()) // Push intake out, and spin
        {
            double statorCurrent = intakeRoller.getStatorCurrent();
            if ((Math.abs(statorCurrent) < Constants.ZUCC_JAM_CURRENT) && !robotHandler.stateHandler.commitingToUnjam)
            {
                // Tune current late for antijam
                intakeRoller.set(Constants.ZUCC_SPEED);


                
                // ---------------------------------------------------------------
                // Worry about this line, and account for it when 4 balls are held
                // ---------------------------------------------------------------
                intakeBeltFront.set(Constants.INTAKE_BELT_FRONT_SPEED);
            }
            else if (!robotHandler.stateHandler.commitingToUnjam)
            {
                robotHandler.stateHandler.commitToUnjam();
                intakeRoller.set(Constants.ZUCC_JAM_SPEED);
            }
            else // If already commited to unjam
            {
                if (robotHandler.stateHandler.lastCommitToUnjamTime + 1 > Timer.getFPGATimestamp())
                    intakeRoller.set(Constants.ZUCC_JAM_SPEED);
                else
                    robotHandler.stateHandler.uncommitToUnjam();
            }
        }
    }
