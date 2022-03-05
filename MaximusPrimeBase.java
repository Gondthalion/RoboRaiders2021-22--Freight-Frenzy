package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MaximusPrimeBase {
    OpMode opMode;
    // Motor declaration
    DcMotor dcmCollection, dcmLift, dcmSpinnerR,
            dcmDrivetrainLF, dcmDrivetrainRF, dcmDrivetrainLB, dcmDrivetrainRB, dcmCapping;
    // Servo declaration
    CRServo crsDelivery;
    Servo crsCapping;
    // IMU setup
    BNO055IMU imu;
    Orientation angles;
    NormalizedColorSensor csCollection;
    // Timers
    ElapsedTime tmrGeneric = new ElapsedTime();
    ElapsedTime tmrTeleop = new ElapsedTime();
    ElapsedTime tmrIMU = new ElapsedTime();
    ElapsedTime tmrEncoderDrive = new ElapsedTime();
    ElapsedTime tmrCarouselTele = new ElapsedTime();
    /*          AUTONOMOUS AND TELEOP VARIABLES         */
    Alliance alliance = Alliance.BLUE1;
    private enum Alliance{
        RED1, RED2, BLUE1, BLUE2
    }
    float IMUReading = 0;
    /*                TELEOP VARIABLES               */
    TingleBombs tingleBombs = TingleBombs.UP_FREIGHT;
    private enum TingleBombs{
        UP_FREIGHT,
        UP_TSE,
        DELIVERING_FREIGHT,
        DELIVERING_TSE
    }
    boolean bCollectedBlock = false;
    int iRestingTicks = 100;
    int iPickingFreightTicks = 300;
    int iPickingTSETicks = 400;
    int iDeliveringFreightTicks = 900;
    int iDeliveringTSETicks = 1200;
    int iTargetTicks = iRestingTicks;

    double crsCappingPosOffset = 0;
    double carouselTimerOffset = 0;
    boolean autoTurnEnabled = false;
    // Variables used in determining drivetrain speed
    double drvTrnSpd = .75;
    boolean upFlag = false;
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    // Variables used in field centric driver code
    int count = 0;
    double[] angleTest = new double[10];
    double average;
    double correct;
    double startingHeadingInRadians = 1.5708;
    /*             AUTONOMOUS VARIABLES             */
    double currentLinearPositionInInches = 0;
    double amountOfVeer = 1;
    double ticksPerInch = 42.8;
    int leftSideEncoderAverage = 0;
    int rightSideEncoderAverage = 0;
    int frontSideEncoderAverage = 0;
    int backSideEncoderAverage = 0;
    double collectionDistanceSensorReading = 0;
    AutonomousTargetLevel autonomousTargetLevel = AutonomousTargetLevel.LOW;
    private enum AutonomousTargetLevel {
        HIGH, MIDDLE, LOW
    }
    double carouselPower = 0;
    // Link classes and run the configuration function
    public MaximusPrimeBase(OpMode theOpMode){
        opMode = theOpMode;
        Configuration();
    }
    /*Teleop Functions*/
    public void OperatorControls() {                                                                // Operator Controls
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            dcmCapping.setPower(opMode.gamepad2.left_trigger + opMode.gamepad2.right_trigger);
        } else {
            dcmCapping.setPower(-opMode.gamepad2.left_trigger - opMode.gamepad2.right_trigger);
        }
        if (opMode.gamepad2.left_bumper) {
            crsCapping.setPosition(0 - crsCappingPosOffset);
        } else if (opMode.gamepad2.right_bumper) {
            crsCapping.setPosition(1 - crsCappingPosOffset);
        }
        if (opMode.gamepad2.dpad_left) {
            crsCappingPosOffset = crsCapping.getPosition();
        }
        // Turn off the automatic the lift if the joystick is used
        // Lift controls.
        dcmLift.setPower(-opMode.gamepad2.left_stick_y);
        // Collection controls.
        dcmCollection.setPower(opMode.gamepad2.right_stick_y);
        // Open/close delivery servo
        if (opMode.gamepad2.y) {
            crsDelivery.setPower(1);
        } else if (opMode.gamepad2.x) {
            crsDelivery.setPower(-1);
        } else {
            crsDelivery.setPower(0);
        }
    }
    public void CarouselTele() {
        if (tmrCarouselTele.seconds()>1) {
            carouselPower = 1;
        } else {
            carouselPower = .35;
        }
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            if (opMode.gamepad1.left_bumper) {
                dcmSpinnerR.setPower(carouselPower);
            } else {
                dcmSpinnerR.setPower(0);
                tmrCarouselTele.reset();
            }
        } else {
            if (opMode.gamepad1.left_bumper) {
                dcmSpinnerR.setPower(-carouselPower);
            } else {
                dcmSpinnerR.setPower(0);
                tmrCarouselTele.reset();
            }
        }
    }
    public void DriverControls() {                                                                  // Driver controls
        CarouselTele();
        TeleIMUTurn();
        // Speed variation for the drivetrain.
        if (opMode.gamepad1.dpad_up){
            upFlag = true;
        } else {
            upFlag = false;
            upPersistent = false;
        }
        if (upFlag && !upPersistent) {
            if (drvTrnSpd < 1){drvTrnSpd += .1;}
            upPersistent = true;
        }
        if (opMode.gamepad1.dpad_down){
            downFlag = true;
        } else {
            downFlag = false;
            downPersistent = false;
        }
        if (downFlag && !downPersistent) {
            if (drvTrnSpd > .1){drvTrnSpd -= .1;}
            downPersistent = true;
        }
    }
    public void AutoCapping() {
        if (opMode.gamepad2.dpad_down) {
            iTargetTicks = iPickingFreightTicks;
        }

        if (opMode.gamepad2.dpad_down || opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_up || opMode.gamepad2.dpad_right) {
            switch (tingleBombs) {
                case UP_FREIGHT:
            }
        }
    }
    void TeleIMUTurn() {                                                                            // Teleop IMU turn
        // Update our current heading
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        IMUReading = (angles.firstAngle);
        // If the dpad is pressed, determine which way to turn
        if (opMode.gamepad1.dpad_right) {
            // Turn off the manual drivetrain movement
            autoTurnEnabled = true;
            // Determine our speed based on how far a way we are from the target
            float power = Math.max(Math.abs(IMUReading)/75, .2f);
            // If we are to the left of the target, turn right
            if (IMUReading > 2) {
                setDrivePowerSides(power, power);
            }
            // If we are to the right of the target, turn left
            else if (IMUReading < -2) {
                setDrivePowerSides(-power, -power);
            }
            // If we are within two degrees of the target, stop
            else {
                stopDrivetrain();
            }
        }
        // If the automatic turn button is not pressed, turn on the manual drivetrain
        else {
            autoTurnEnabled = false;
        }
    }
    public void UpdateDriveTrain() {                                                                // Field-centric driver code
        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double speed = Math.hypot
                (opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2
                (opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle - (Math.PI)/2 - startingHeadingInRadians;
        double turnPower = opMode.gamepad1.right_stick_x;
        if(turnPower == 0){
            if (count < 10) {
                angleTest[count] = angle;
                angleTest[count] = angle;
                count++;
            }
            else {
                average = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0]
                        + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8]
                        + angleTest[9])/10;
                if(average > angle){
                    correct = average - angle;
                    angle = angle + correct;
                }
                else if(angle > average){
                    correct = angle - average;
                    angle = angle - correct;
                }
                count = 0;
            }
        }
        if (!autoTurnEnabled) {
            dcmDrivetrainLF.setPower((((speed * -(Math.sin(angle)) + turnPower))) * drvTrnSpd);
            dcmDrivetrainLB.setPower((((speed * -(Math.cos(angle)) + turnPower))) * drvTrnSpd);
            dcmDrivetrainRF.setPower((((speed * (Math.cos(angle))) + turnPower)) * drvTrnSpd);
            dcmDrivetrainRB.setPower((((speed * (Math.sin(angle))) + turnPower)) * drvTrnSpd);
        }
    }
    /*Autonomous Functions*/
    public void EncoderDrive(double inToMove, double maxSpeedDistance,                              // Encoder drive
                             double minSpeed, float timeOut, int headingOffset) {
        // Reset the stall out timer
        tmrEncoderDrive.reset();
        // Reset the encoders before moving
        ResetEncoders();
        // Find the average of the encoders on the left side of the drivetrain
        leftSideEncoderAverage = (((dcmDrivetrainLB.getCurrentPosition()) +
                (dcmDrivetrainLF.getCurrentPosition()))/2);
        // Find the average of the encoders on the right side of the drivetrain
        rightSideEncoderAverage = -(((dcmDrivetrainRB.getCurrentPosition()) +
                (dcmDrivetrainRF.getCurrentPosition()))/2);
        // We divide a number by this variable, so this make division by zero impossible
        if (leftSideEncoderAverage == 0) {
            leftSideEncoderAverage = 1;
        }
        // We divide a number by this variable, so this make division by zero impossible
        if (rightSideEncoderAverage == 0) {
            rightSideEncoderAverage = 1;
        }
        // Find the average of the entire drivetrain
        currentLinearPositionInInches =
                ((leftSideEncoderAverage + rightSideEncoderAverage)/2)/ticksPerInch;

        // The important part says to keep moving until the
        // current position is grater than the target position
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentLinearPositionInInches) <
                Math.abs(inToMove) && tmrEncoderDrive.seconds() < timeOut) {
            UpdateColorSensor();
            if (collectionDistanceSensorReading < 1.35) {
                dcmCollection.setPower(0);
                bCollectedBlock = true;
            }
            // Find the average of the encoders on the left side of the drivetrain
            leftSideEncoderAverage = (((dcmDrivetrainLB.getCurrentPosition()) +
                    (dcmDrivetrainLF.getCurrentPosition()))/2);
            // Find the average of the encoders on the right side of the drivetrain
            rightSideEncoderAverage = -(((dcmDrivetrainRB.getCurrentPosition()) +
                    (dcmDrivetrainRF.getCurrentPosition()))/2);
            // Updating current pos.
            currentLinearPositionInInches =
                    ((leftSideEncoderAverage + rightSideEncoderAverage)/2)/ticksPerInch;
            // We divide a number by this variable, so this make division by zero impossible
            if (leftSideEncoderAverage == 0) {
                leftSideEncoderAverage = 1;
            }
            // We divide a number by this variable, so this make division by zero impossible
            if (rightSideEncoderAverage == 0) {
                rightSideEncoderAverage = 1;
            }

            // Constantly updating the power to the motors based on how far we have to move.
            double power = Math.max(Math.abs(currentLinearPositionInInches - inToMove)
                    /maxSpeedDistance, minSpeed);
            // Update our current heading
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            // Calculate how much we are veering using the Control Hub's IMU sensor
            amountOfVeer = Math.min(((IMUReading - headingOffset) / 20), .4);
            // If we are veering so that we are spinning clockwise,
            // subtract power from the appropriate motors
            if (amountOfVeer > 0) {
                // If we are moving forward, apply positive and
                // negative motor powers to the appropriate motors
                if (inToMove >= 0) {
                    setDrivePowerSides(power, (-power + amountOfVeer));
                }
                // If we are moving backwards, apply positive and
                // negative motor powers to the appropriate motors
                else if (inToMove < 0) {
                    setDrivePowerSides((-power + amountOfVeer), power);
                }
            }
            // If we are veering so that we are spinning counter-clockwise,
            // subtract power from the appropriate motors
            else if (amountOfVeer <= 0){
                // If we are moving forward, apply positive and
                // negative motor powers to the appropriate motors
                if (inToMove >= 0) {
                    setDrivePowerSides((power + amountOfVeer), -power);
                }
                // If we are moving backwards, apply positive and
                // negative motor powers to the appropriate motors
                else if (inToMove < 0) {
                    setDrivePowerSides(-power, (power + amountOfVeer));
                }
            }
        }
        // Stopping the drivetrain after reaching the target.
        stopDrivetrain();
        // Reset encoders after movement
        ResetEncoders();
    }
    public void EncoderDriveSideways(double inToMove, double maxSpeedDistance,                      // Encoder Drive Sideways
                                     double minSpeed, float timeOut, int headingOffset) {
        // SEE ENCODERDRIVE FOR DOCUMENTATION
        tmrEncoderDrive.reset();
        ResetEncoders();

        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        IMUReading = (angles.firstAngle);

        frontSideEncoderAverage = (((dcmDrivetrainRF.getCurrentPosition()) +
                (dcmDrivetrainLF.getCurrentPosition()))/2);
        rightSideEncoderAverage = -(((dcmDrivetrainRB.getCurrentPosition()) +
                (dcmDrivetrainLB.getCurrentPosition()))/2);

        if (frontSideEncoderAverage == 0) {
            frontSideEncoderAverage = 1;
        }
        if (backSideEncoderAverage == 0) {
            backSideEncoderAverage = 1;
        }
        currentLinearPositionInInches =
                ((frontSideEncoderAverage + backSideEncoderAverage)/2)/ticksPerInch;
        while (((LinearOpMode)opMode).opModeIsActive() && Math.abs(currentLinearPositionInInches) <
                Math.abs(inToMove) && tmrEncoderDrive.seconds() < timeOut) {
            frontSideEncoderAverage = (((dcmDrivetrainRF.getCurrentPosition()) +
                    (dcmDrivetrainLF.getCurrentPosition()))/2);
            rightSideEncoderAverage = -(((dcmDrivetrainRB.getCurrentPosition()) +
                    (dcmDrivetrainLB.getCurrentPosition()))/2);
            currentLinearPositionInInches =
                    ((frontSideEncoderAverage + backSideEncoderAverage)/2)/ticksPerInch;
            if (frontSideEncoderAverage == 0) {
                frontSideEncoderAverage = 1;
            }
            if (backSideEncoderAverage == 0) {
                backSideEncoderAverage = 1;
            }
            double power = Math.max(Math.abs(currentLinearPositionInInches -inToMove)/maxSpeedDistance, minSpeed);
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            amountOfVeer = Math.min(((IMUReading - headingOffset) / 15), .4);
            if (amountOfVeer > 0) {
                if (inToMove >= 0) {
                    dcmDrivetrainLF.setPower(power);
                    dcmDrivetrainLB.setPower(-power + amountOfVeer);
                    dcmDrivetrainRF.setPower(power);
                    dcmDrivetrainRB.setPower(-power + amountOfVeer);
                } else if (inToMove < 0) {
                    dcmDrivetrainLF.setPower(-power + amountOfVeer);
                    dcmDrivetrainLB.setPower(power);
                    dcmDrivetrainRF.setPower(-power + amountOfVeer);
                    dcmDrivetrainRB.setPower(power);
                }
            } else if (amountOfVeer <= 0){
                if (inToMove >= 0) {
                    dcmDrivetrainLF.setPower(power - amountOfVeer);
                    dcmDrivetrainLB.setPower(-power);
                    dcmDrivetrainRF.setPower(power);
                    dcmDrivetrainRB.setPower(-power + amountOfVeer);
                } else if (inToMove < 0) {
                    dcmDrivetrainLF.setPower(-power + amountOfVeer);
                    dcmDrivetrainLB.setPower(power);
                    dcmDrivetrainRF.setPower(-power + amountOfVeer);
                    dcmDrivetrainRB.setPower(power);
                }
            }
        }
        stopDrivetrain();
        ResetEncoders();
    }
    void IMUTurn(float targetAngle, String leftOrRight,                                             // IMU Turn
                 float minSpeed, float maxSpeedAngle, float timeOut) {
        // Reset the stall out timer
        tmrIMU.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& tmrIMU.seconds() < timeOut) {
            // Update our current heading while turning
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            // Calculating the motor powers based on how far away we are form the target angle
            float power = Math.max(Math.abs(IMUReading -targetAngle)/maxSpeedAngle, minSpeed);
            // If we are close enough to our target angle, break out of the while loop
            if (Math.abs(IMUReading) < Math.abs(targetAngle)) {
                if ((Math.abs(IMUReading) - Math.abs(targetAngle)) > -2){
                    break;
                }
            }
            // If we are turning left, apply positive and negative powers to the appropriate motors
            if (leftOrRight.equals("left") || leftOrRight.equals("l")){
                // Actually applying the power
                setDrivePowerSides(-power, -power);
            }
            // If we are turning right, apply positive and negative powers to the appropriate motors
            else {
                // Actually applying the power
                setDrivePowerSides(power, power);
            }
        }
        // Stopping the motors once we completed our turn
        stopDrivetrain();
    }
    public void RedOne() {                                                                          // Red One
        // Move backward to barcode
        EncoderDrive(-11,25,.3,2,0);
        // Strafe to the wall
        EncoderDriveSideways(18, 35,
                .2, 2, 0);
        // Move forward to carousel
        EncoderDrive(5.5,27,.3,2,0);
        // Deliver the duck
        CarouselAuto();
        // Move backward past Storage Unit
        EncoderDrive(-41, 49, .2, 2 , 2);
        // Strafe away from the wall
        EncoderDriveSideways(-5, 12,
                .2, 2, 0);
        // Turn towards Alliance Shipping Hub
        IMUTurn(-88, "r",.1f, 270, 3);
        // Move forward to Alliance Shipping Hub
        EncoderDrive(-24.5, 37, .2, 2, -83);
        // Deliver the Pre-Loaded Box leaving the lift raised
        DeliverBlock();
        // Back up near the wall
        EncoderDrive(27.5, 37, .2, 2, -90);
        // Turn to be parallel with the wall
        IMUTurn(1, "l",.2f, 270,3);
        // Strafe to the wall
        EncoderDriveSideways(8, 11,
                .2, 2, 0);
        // Move forward into Storage Unit
        EncoderDrive(14, 22, .2, 2,1);
        // Lower the lift
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
        }
        dcmLift.setPower(0);
        opMode.telemetry.addData("Heading: ", IMUReading);
        opMode.telemetry.update();
    }
    public void BlueOne() {                                                                         // Blue One
        // Move backward to barcode
        EncoderDrive(-11,25,.3,2,0);
        IMUTurn(-88, "r",.1f, 270,3);
        EncoderDrive(-22,25,.3,2,-88);
        IMUTurn(2, "l",.1f, 270,3);
        EncoderDriveSideways(-5, 20,
                .2, 2, 0);
        EncoderDrive(8, 10, .2, 2 , 0);
        // Move forward to carousel
        // Deliver the duck
        CarouselAuto();
        // Move backward past Storage Unit
        EncoderDrive(-38, 49, .2, 2 , -2);
        // Strafe away from the wall
        EncoderDriveSideways(5, 20,
                .2, 2, 0);
        // Turn towards Alliance Shipping Hub
        IMUTurn(90, "l",.1f, 270,3);
        // Move forward to Alliance Shipping Hub
        EncoderDrive(-25.5, 37, .2, 2, 92);
        // Deliver the Pre-Loaded Box leaving the lift raised
        DeliverBlock();
        // Back up near the wall
        EncoderDrive(26.5, 37, .2, 2, 90);
        // Turn to be parallel with the wall
        IMUTurn(6, "r",.2f, 270,3);
        // Strafe to the wall
        EncoderDriveSideways(-8, 25,
                .2, 2, 3);
        // Move forward into Storage Unit
        EncoderDrive(11, 22, .2, 2,3);
        // Lower the lift
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
        }
        dcmLift.setPower(0);
        opMode.telemetry.addData("Heading: ", IMUReading);
        opMode.telemetry.update();
    }
    public void RedTwo() {                                                                          // Red Two
        // Move backward to barcode
        EncoderDrive(-5, 20, .2, 2, 0);
        //  ColorSensorReadings();
        // Strafe to the front of the Alliance Shipping Hub
        // EncoderDriveSideways(7, 34, .2, 2, 0);
        IMUTurn(20, "l", .3f, 270,3);
        // Move backward to the Alliance Shipping Hub
        EncoderDrive(-19, 16, .2, 2, 20);
        // Deliver the Pre-Loaded freight
        DeliverBlock();
        // Move away from the Alliance Shipping Hub
        EncoderDrive(15, 15, .2,2,20);
        // Lower the lift
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
        }
        dcmLift.setPower(0);
        // Turn parallel to the wall
        IMUTurn(90, "l",.1f, 270,3);
        // Strafe into the wall
        EncoderDriveSideways(6, 13, .2, 2, 90);
        // Move into the Warehouse
        dcmCollection.setPower(1);
        Sleep(200);
        dcmCollection.setPower(-1);
        Sleep(200);
        dcmCollection.setPower(0);
        EncoderDrive(47, 48, .4, 4, 88);
        dcmCollection.setPower(-1);
        EncoderDrive(-10, 92, .2, 3, 88);
        if (bCollectedBlock) {
            dcmCollection.setPower(-1);
            Sleep(500);
            EncoderDrive(-53, 55, .2, 3, 92);
            dcmCollection.setPower(0);
            EncoderDriveSideways(-5, 15, .2, 2, 90);
            IMUTurn(1, "r", .1f, 270, 3);
            EncoderDrive(-13, 26, .2, 2, 0);
            if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
            DeliverBlock();
            EncoderDrive(10, 15, .2, 2, 0);
            IMUTurn(90, "l", .1f, 270, 3);
            EncoderDriveSideways(8, 18, .2, 2, 88);
            EncoderDrive(69, 40, .2, 2, 88);
        } else {
            dcmCollection.setPower(0);
            EncoderDrive(15, 40, .2, 2, 86);
            // Strafe away from the wall
            opMode.telemetry.addData("Heading: ", IMUReading);
            opMode.telemetry.update();
        }
    }
    public void BlueTwo() {                                                                         // Blue Two
        // Move backward to barcode
        EncoderDrive(-5, 10, .2, 2, 0);
        //  ColorSensorReadings();
        // Strafe to the front of the Alliance Shipping Hub
        // EncoderDriveSideways(7, 34, .2, 2, 0);
        IMUTurn(-80, "r", .3f, 120,3);
        // Move backward to the Alliance Shipping Hub
        EncoderDrive(-28, 29, .2, 2, -87);
        IMUTurn(-7, "l", .3f, 120,3);
        EncoderDrive(-19.5, 18, .2, 2, 0);
        // Deliver the Pre-Loaded freight
        DeliverBlock();
        // Move away from the Alliance Shipping Hub
        EncoderDrive(17, 16, .2,2,0);
        // Lower the lift
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
        }
        dcmLift.setPower(0);
        // Turn parallel to the wall
        IMUTurn(-85, "r",.1f, 120,3);
        // Strafe into the wall
        EncoderDriveSideways(-5, 6, .2, 2, -88);
        // Move into the Warehouse
        dcmCollection.setPower(1);
        Sleep(200);
        dcmCollection.setPower(-1);
        Sleep(200);
        dcmCollection.setPower(0);
        EncoderDrive(60, 40, .4, 4, -87);
        dcmCollection.setPower(-1);
        EncoderDrive(-10, 92, .2, 3, -92);
        dcmCollection.setPower(-1);
        Sleep(500);
        if (bCollectedBlock) {
            EncoderDrive(-52, 40, .2, 3, -92);
            dcmCollection.setPower(0);
            EncoderDriveSideways(5, 15, .2, 2, -90);
            IMUTurn(-2, "l", .1f, 120, 3);
            EncoderDrive(-13, 26, .2, 2, 0);
            if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
            DeliverBlock();
            EncoderDrive(10, 10, .2, 2, 0);
            IMUTurn(-85, "r", .1f, 120, 3);
            EncoderDriveSideways(-15, 25, .2, 2, -86);
            EncoderDrive(60, 40, .2, 2, -86);
        } else {
            dcmCollection.setPower(0);
            EncoderDrive(15, 40, .2, 2, -86);
            // Strafe away from the wall
            opMode.telemetry.addData("Heading: ", IMUReading);
            opMode.telemetry.update();
        }
    }
    public void CarouselAuto() {                                                                    // Carousel Auto
        // If we are red
        if (alliance == Alliance.RED1 || alliance == Alliance.RED2) {
            // Turn on the carousel spinner for 4.5 seconds
            dcmSpinnerR.setPower(-.3);
            Sleep(5000);
            // Stop the motors
            dcmSpinnerR.setPower(0);
        }
        // If we are blue
        else if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            // Turn on the carousel spinner for 4.5 seconds
            dcmSpinnerR.setPower(.3);
            Sleep(5000);
            // Stop the motors
            dcmSpinnerR.setPower(0);
        }
    }
    public void DeliverBlock() {                                                                    // Deliver Block
        // Reset the lift's home position
        dcmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // If we should deliver into the high goal
        if (autonomousTargetLevel == AutonomousTargetLevel.HIGH) {
            // Raise the lift.
            dcmLift.setPower(1);
            while (dcmLift.getCurrentPosition() < 2150 && ((LinearOpMode)opMode).opModeIsActive()) {
            }
            dcmLift.setPower(0);
            // Deliver block
            crsDelivery.setPower(1);
            Sleep(2250);
            crsDelivery.setPower(0);
            // Stop the lift
            dcmLift.setPower(0);
        }
        // If we should deliver into the high goal
        else if (autonomousTargetLevel == AutonomousTargetLevel.MIDDLE) {
            // Raise the lift
            dcmLift.setPower(1);
            while (dcmLift.getCurrentPosition() < 1200 && ((LinearOpMode)opMode).opModeIsActive()) {
            }
            dcmLift.setPower(0);
            // Deliver block
            crsDelivery.setPower(1);
            Sleep(2250);
            crsDelivery.setPower(0);
            // Stop the lift
            dcmLift.setPower(0);
        }
        // If we should deliver into the high goal
        else if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
            // Raise the lift
            dcmLift.setPower(1);
            while (dcmLift.getCurrentPosition() < 500 && ((LinearOpMode)opMode).opModeIsActive()) {
            }
            dcmLift.setPower(0);
            // Deliver block
            crsDelivery.setPower(1);
            Sleep(2250);
            crsDelivery.setPower(0);
            // Stop the lift
            dcmLift.setPower(0);
        }
    }
    public void Configuration() {                                                                   // Configuration
        dcmCollection = opMode.hardwareMap.dcMotor.get("collectionM");
        dcmLift = opMode.hardwareMap.dcMotor.get("liftM");
        dcmSpinnerR = opMode.hardwareMap.dcMotor.get("rSpinnerM");
        dcmDrivetrainLF = opMode.hardwareMap.dcMotor.get("lfDrvtrnM");
        dcmDrivetrainRF = opMode.hardwareMap.dcMotor.get("rfDrvtrnM");
        dcmDrivetrainLB = opMode.hardwareMap.dcMotor.get("lbDrvtrnM");
        dcmDrivetrainRB = opMode.hardwareMap.dcMotor.get("rbDrvtrnM");
        dcmCapping = opMode.hardwareMap.dcMotor.get("dcmCapping");
        this.crsDelivery = opMode.hardwareMap.get(CRServo.class, "deliveryS");
        this.crsCapping = opMode.hardwareMap.get(Servo.class, "cappingS");
        csCollection = opMode.hardwareMap.
                get(NormalizedColorSensor.class, "collectionColorSensor");

        dcmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmCapping.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmSpinnerR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmDrivetrainLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmDrivetrainRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmDrivetrainLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcmDrivetrainRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void UpdateColorSensor() {
        collectionDistanceSensorReading = ((DistanceSensor) csCollection).getDistance(DistanceUnit.INCH);
    }
    public void Telemetry() {                                                                       // Telemetry
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
        opMode.telemetry.addData("Teleop timer: ", tmrTeleop);
        opMode.telemetry.addData("Carousel timer: ", tmrCarouselTele);
        opMode.telemetry.addData("Carousel timer adj: ", carouselTimerOffset);
        opMode.telemetry.addData("Lift: ", dcmLift.getCurrentPosition());
        UpdateColorSensor();
        opMode.telemetry.addData("Collection: ", collectionDistanceSensorReading);
        opMode.telemetry.update();
    }
    public void AllianceDetermination() {                                                           // Alliance determination
        // Choosing our starting position for autonomous and teleop.
        // X for Blue 1, A for Blue 2, etc.
        if (opMode.gamepad1.x) {
            alliance = Alliance.BLUE1;
        } else if (opMode.gamepad1.a) {
            alliance = Alliance.BLUE2;
        } else if (opMode.gamepad1.b) {
            alliance = Alliance.RED1;
        } else if (opMode.gamepad1.y) {
            alliance = Alliance.RED2;
        }
        // Telemeter the position currently selected
        opMode.telemetry.addData("Starting auto position", alliance);
        opMode.telemetry.update();
    }
    public void ResetEncoders() {
        // Function to reset encoders
        dcmDrivetrainLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmDrivetrainRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmDrivetrainLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmDrivetrainRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmDrivetrainLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcmDrivetrainRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Autonomous() {
        // Play the autonomous based on what path was chosen
        if (alliance == Alliance.RED1) {
            RedOne();
        } else if (alliance == Alliance.BLUE1) {
            BlueOne();
        } else if (alliance == Alliance.RED2) {
            RedTwo();
        } else if (alliance == Alliance.BLUE2) {
            BlueTwo();
        }
    }
    boolean IsInitialized() {
        return !((LinearOpMode)opMode).isStarted() && !((LinearOpMode)opMode).isStopRequested();
    }
    public void Sleep(long ms) {
        if(((LinearOpMode)opMode).opModeIsActive()){
            ((LinearOpMode)opMode).sleep(ms);
        }
    }
    public void setDrivePowerSides(double motorPowerL, double motorPowerR) {
        dcmDrivetrainLF.setPower(motorPowerL);
        dcmDrivetrainLB.setPower(motorPowerL);
        dcmDrivetrainRF.setPower(motorPowerR);
        dcmDrivetrainRB.setPower(motorPowerR);
    }
    public void stopDrivetrain() {
        dcmDrivetrainLF.setPower(0);
        dcmDrivetrainLB.setPower(0);
        dcmDrivetrainRF.setPower(0);
        dcmDrivetrainRB.setPower(0);
    }
    public void ResetHeading(){ // Resets the imu heading by adding/subtracting from itself
        if (opMode.gamepad1.b){ // In case something goes wrong, driver can reposition the robot and reset the heading during teleop
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            startingHeadingInRadians = (Math.toRadians(IMUReading) + 1.5708);
        }
    }
    public void RetrieveShippingElementPosition() {
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            if (MaximusPrimeAuto.SkystoneDeterminationPipeline.blueSquareActivated) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else if (MaximusPrimeAuto.SkystoneDeterminationPipeline.greenSquareActivated) {
                autonomousTargetLevel = AutonomousTargetLevel.HIGH;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
        } else if (alliance == Alliance.RED1 || alliance == Alliance.RED2) {
            if (MaximusPrimeAuto.SkystoneDeterminationPipeline.blueSquareActivated) {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            } else if (MaximusPrimeAuto.SkystoneDeterminationPipeline.greenSquareActivated) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.HIGH;
            }
        }
        opMode.telemetry.addData("Pos", autonomousTargetLevel);
        opMode.telemetry.update();
    }
}