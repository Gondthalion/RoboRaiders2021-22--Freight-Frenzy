package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MaximusPrimeBase {
    OpMode opMode;
    // This kind of creates the base for the motors to be set up on.
    // dcm stands for direct current motor: which is the type we are using
    DcMotor dcmCollection, dcmLift, dcmSpinnerR,
            dcmDrivetrainLF, dcmDrivetrainRF, dcmDrivetrainLB, dcmDrivetrainRB, dcmCapping;
    // This is just like the motor setup. cr stands for continuous rotation
    CRServo crsDelivery, crsCapping;
    // This is the base for the lights on our robot
    RevBlinkinLedDriver daLights;
    // IMU setup. The first line sets up the actual IMU device.
    // I'm not entirely certain what the second one sets up
    BNO055IMU imu;
    Orientation angles;
    // This is the base for the color sensor for the lights on our robot
    NormalizedColorSensor csCollection;
    // These lines initialize timers
    ElapsedTime tmrGeneric = new ElapsedTime();
    ElapsedTime tmrTeleop = new ElapsedTime();
    ElapsedTime tmrIMU = new ElapsedTime();
    ElapsedTime tmrEncoderDrive = new ElapsedTime();
    ElapsedTime tmrCarouselTele = new ElapsedTime();
    /*          AUTONOMOUS AND TELEOP VARIABLES         */
    // This is an enumerator. You can store any number of variables in them.
    // They are useful because they are much easier to read than an int or series of bools.
    // This one stores what autonomous path we should run.
    Alliance alliance = Alliance.BLUE1;
    public enum Alliance{
        RED1, RED2, RED2_NO_BLOCK, BLUE1, BLUE2, BLUE2_NO_BLOCK
    }
    // The heading from the IMU
    float IMUReading = 0;
    /*                TELEOP VARIABLES               */
    boolean bCollectedBlock = false;                // To turn on lights if block collected
    double dCappingSpeed = 1;                       // Power for capping arm
    boolean upFlagCapping = false;                  // Variable for changing capping power
    boolean upPersistentCapping = false;
    boolean downFlagCapping = false;
    boolean downPersistentCapping = false;
    boolean autoTurnEnabled = false;                // Disable driver controls if auto turning (in teleop)
    double drvTrnSpd = .75;                         // Power for capping arm
    boolean upFlag = false;                         // Variable for changing drivetrain power
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    int count = 0;                                  // Variable used in field centric driver code
    double[] angleTest = new double[10];
    double average;
    double correct;
    // YOU CAN CHANGE THIS TO CHANGE WHAT ANGLE THE FIELD CENTRIC CODE THINKS IS ZERO
    double startingHeadingInRadians = 1.5708;       // THIS IS IN RADIANS.
    /*             AUTONOMOUS VARIABLES             */
    double currentLinearPositionInInches = 0;       // Pos for encoder drives
    double amountOfVeer = 1;                        // Veer for encoder drives
    double ticksPerInch = 42.8;                     // Roughly motor tics per in on our robot (including gears)
    int leftSideEncoderAverage = 0;                 // Exactly what it sounds like. Used for encoder drives
    int rightSideEncoderAverage = 0;
    int frontSideEncoderAverage = 0;
    int backSideEncoderAverage = 0;
    double collectionDistanceSensorReading = 0;     // Used for lights
    // This stores the desired level of the alliance shipping hub to score on in auto
    AutonomousTargetLevel autonomousTargetLevel = AutonomousTargetLevel.LOW;
    private enum AutonomousTargetLevel {
        HIGH, MIDDLE, LOW
    }
    double carouselPower = 0;                       // Carousel power. Like, I feel like that's pretty obvious
    // Link classes and run the configuration function
    public MaximusPrimeBase(OpMode theOpMode){
        opMode = theOpMode;
        Configuration();
    }
    /* Teleop Functions */
    /* These are the operator controls. They are really simple, but control about half of the robot */
    public void OperatorControls() {                                                                        // Operator Controls
        // Setting capping arm speed
        dcmCapping.setPower(opMode.gamepad2.left_trigger*dCappingSpeed -
                opMode.gamepad2.right_trigger*dCappingSpeed);
        // Setting capping servo speed
        if (opMode.gamepad2.left_bumper) {
            crsCapping.setPower(-.25);
        } else if (opMode.gamepad2.right_bumper) {
            crsCapping.setPower(.25);
        } else {
            crsCapping.setPower(0);
        }
        // Setting lift speed
        dcmLift.setPower(-opMode.gamepad2.left_stick_y);
        // Setting collection speed
        dcmCollection.setPower(opMode.gamepad2.right_stick_y);
        // Setting deliver servo speed
        if (opMode.gamepad2.y) {
            crsDelivery.setPower(1);
        } else if (opMode.gamepad2.x) {
            crsDelivery.setPower(-1);
        } else {
            crsDelivery.setPower(0);
        }
        // Capping arm speed variability
        if (opMode.gamepad2.dpad_up){
            upFlagCapping = true;
        } else {
            upFlagCapping = false;
            upPersistentCapping = false;
        }
        if (upFlagCapping && !upPersistentCapping) {
            if (dCappingSpeed < 1){dCappingSpeed += .1;}
            upPersistentCapping = true;
        }
        if (opMode.gamepad2.dpad_down){
            downFlagCapping = true;
        } else {
            downFlagCapping = false;
            downPersistentCapping = false;
        }
        if (downFlagCapping && !downPersistentCapping) {
            if (dCappingSpeed > .1){dCappingSpeed -= .1;}
            downPersistentCapping = true;
        }
    }
    /* This function changes the color of the lights based on different criteria */
    public void Lights() {
        UpdateColorSensor();
        if (collectionDistanceSensorReading < 1.35) {
            tmrGeneric.reset();
            bCollectedBlock = true;
        } else if (tmrGeneric.seconds() > 1) {
            bCollectedBlock = false;
        }
        if (bCollectedBlock) {
            daLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (tmrTeleop.seconds() > 85 && tmrTeleop.seconds() < 90) {
            daLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
        } else if (tmrTeleop.seconds() > 90) {
            daLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            daLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }
    /* These are the driver controls. They are really simple, and control the carousel and tele imu turn */
    public void DriverControls() {                                                                  // Driver controls
        // Sets the carousel motor power high if the right button is pressed
        // and low if the left is pressed
        if (alliance == Alliance.BLUE1 || alliance == Alliance.BLUE2) {
            if (opMode.gamepad1.left_bumper) {
                dcmSpinnerR.setPower(.3);
            } else if (opMode.gamepad1.right_bumper) {
                dcmSpinnerR.setPower(1);
            } else {
                dcmSpinnerR.setPower(0);
            }
        } else {
            if (opMode.gamepad1.left_bumper) {
                dcmSpinnerR.setPower(-.3);
            } else if (opMode.gamepad1.right_bumper) {
                dcmSpinnerR.setPower(-1);
            } else {
                dcmSpinnerR.setPower(0);
            }
        }
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
        // Speed variation for the drivetrain
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
            if (drvTrnSpd > .1){
                drvTrnSpd -= .1;
            }
            downPersistent = true;
        }
    }
    /* This is where the field centric code is actually based.
    I didn't make it, but it's dope */
    public void UpdateDriveTrain() {                                                                // Field-centric driver code
        // Update the imu heading
        angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        // Fancy math to map the value of the x and y parts of the joystick as polar coordinates.
        // Polar coordinates are a angle value and distance value.
        // In this case, angle determines the robot's movement angle and the distance determines power
        double speed = Math.hypot
                (opMode.gamepad1.left_stick_x, opMode.gamepad1.left_stick_y);
        double angle = Math.atan2
                (opMode.gamepad1.left_stick_y, opMode.gamepad1.left_stick_x) - (Math.PI/4);
        angle += angles.firstAngle - (Math.PI)/2 - startingHeadingInRadians;
        // This uses the position of the other joystick to find how fast it should be turning
        double turnPower = opMode.gamepad1.right_stick_x;
        // Calculates the power to apply to each motor based on the math above
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
        // If the robot is not auto turning, moved based on joystick input
        if (!autoTurnEnabled) {
            dcmDrivetrainLF.setPower((((speed * -(Math.sin(angle)) + turnPower))) * drvTrnSpd);
            dcmDrivetrainLB.setPower((((speed * -(Math.cos(angle)) + turnPower))) * drvTrnSpd);
            dcmDrivetrainRF.setPower((((speed * (Math.cos(angle))) + turnPower)) * drvTrnSpd);
            dcmDrivetrainRB.setPower((((speed * (Math.sin(angle))) + turnPower)) * drvTrnSpd);
        }
    }
    /*Autonomous Functions*/
    /* Encoder drive for autonomous. It's FAR from perfect.
    Give yourself more than a week to figure out the logic you want to use */
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
    /* This is seriously crappy programming. There is no need to have this be a separate function.
    Anyway, see the encoder drive function since it's pretty much the same exact thing */
    public void EncoderDriveSideways(double inToMove, double maxSpeedDistance,                      // Encoder Drive Sideways
                                     double minSpeed, float timeOut, int headingOffset) {
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
    /* IMU turn for autonomous. It uses the IMU sensor built into the control hub to turn */
    void IMUTurn(float targetAngle, String leftOrRight,                                             // IMU Turn
                 float minSpeed, float maxSpeedAngle) {
        // Reset the stall out timer
        tmrIMU.reset();
        while (((LinearOpMode)opMode).opModeIsActive()&& tmrIMU.seconds() < 3) {
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
    /* Autonomous paths */
    public void RedOne() {                                                                          // Red One
        EncoderDrive(-11,25,.3,2,0);
        EncoderDriveSideways(18, 35,
                .2, 2, 0);
        EncoderDrive(5.5,27,.3,2,0);
        CarouselAuto();
        EncoderDrive(-41, 49, .2, 2 , 2);
        EncoderDriveSideways(-5, 12,
                .2, 2, 0);
        IMUTurn(-88, "r",.1f, 270);
        EncoderDrive(-24.5, 37, .2, 2, -83);
        DeliverBlock();
        EncoderDrive(27.5, 37, .2, 2, -90);
        IMUTurn(1, "l",.2f, 270);
        EncoderDriveSideways(8, 11,
                .2, 2, 0);
        EncoderDrive(14, 22, .2, 2,1);
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
            tmrGeneric.seconds();
        }
        dcmLift.setPower(0);
        opMode.telemetry.addData("Heading: ", IMUReading);
        opMode.telemetry.update();
    }
    public void BlueOne() {                                                                         // Blue One
        // Move backward to barcode
        EncoderDrive(-11,25,.3,2,0);
        IMUTurn(-88, "r",.1f, 270);
        EncoderDrive(-22,25,.3,2,-88);
        IMUTurn(2, "l",.1f, 270);
        EncoderDriveSideways(-5, 20,
                .2, 2, 0);
        EncoderDrive(9, 10, .2, 2 , 0);
        CarouselAuto();
        EncoderDrive(-42.5, 49, .2, 2 , -2);
        EncoderDriveSideways(5, 20,
                .2, 2, 0);
        IMUTurn(90, "l",.1f, 270);
        EncoderDrive(-27, 37, .2, 2, 92);
        DeliverBlock();
        EncoderDrive(26.5, 37, .2, 2, 90);
        IMUTurn(6, "r",.2f, 270);
        EncoderDriveSideways(-8, 25,
                .2, 2, 3);
        EncoderDrive(13, 22, .2, 2,3);
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
            tmrGeneric.seconds();
        }
        dcmLift.setPower(0);
        opMode.telemetry.addData("Heading: ", IMUReading);
        opMode.telemetry.update();
    }
    public void RedTwo() {                                                                          // Red Two
        EncoderDrive(-5, 20, .2, 2, 0);
        IMUTurn(20, "l", .3f, 270);
        EncoderDrive(-19, 16, .2, 2, 20);
        DeliverBlock();
        EncoderDrive(15, 15, .2,2,20);
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
            tmrGeneric.seconds();
        }
        dcmLift.setPower(0);
        IMUTurn(90, "l",.1f, 270);
        EncoderDriveSideways(6, 13, .2, 2, 90);
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
            IMUTurn(1, "r", .1f, 270);
            EncoderDrive(-13, 26, .2, 2, 0);
            if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
            DeliverBlock();
            EncoderDrive(10, 15, .2, 2, 0);
            IMUTurn(90, "l", .1f, 270);
            EncoderDriveSideways(8, 18, .2, 2, 88);
            EncoderDrive(69, 40, .2, 2, 88);
        } else {
            dcmCollection.setPower(0);
            EncoderDrive(15, 40, .2, 2, 86);
            opMode.telemetry.addData("Heading: ", IMUReading);
            opMode.telemetry.update();
        }
    }
    public void BlueTwo() {                                                                         // Blue Two
        EncoderDrive(-5, 10, .2, 2, 0);
        IMUTurn(-80, "r", .3f, 120);
        EncoderDrive(-28, 29, .2, 2, -87);
        IMUTurn(-7, "l", .3f, 120);
        EncoderDrive(-19.5, 18, .2, 2, 0);
        DeliverBlock();
        EncoderDrive(17, 16, .2,2,0);
        dcmLift.setPower(-1);
        while (dcmLift.getCurrentPosition() > 100 && ((LinearOpMode)opMode).opModeIsActive()) {
            tmrGeneric.seconds();
        }
        dcmLift.setPower(0);
        IMUTurn(-85, "r",.1f, 120);
        EncoderDriveSideways(-5, 6, .2, 2, -88);
        dcmCollection.setPower(1);
        Sleep(200);
        dcmCollection.setPower(-1);
        Sleep(200);
        dcmCollection.setPower(0);
        EncoderDrive(60, 40, .4, 4, -81);
        dcmCollection.setPower(-1);
        EncoderDrive(-10, 92, .2, 3, -92);
        dcmCollection.setPower(-1);
        Sleep(500);
        if (bCollectedBlock) {
            EncoderDrive(-52, 40, .2, 3, -92);
            dcmCollection.setPower(0);
            EncoderDriveSideways(5, 15, .2, 2, -90);
            IMUTurn(-2, "l", .1f, 120);
            EncoderDrive(-13, 26, .2, 2, 0);
            if (autonomousTargetLevel == AutonomousTargetLevel.LOW) {
                autonomousTargetLevel = AutonomousTargetLevel.MIDDLE;
            } else {
                autonomousTargetLevel = AutonomousTargetLevel.LOW;
            }
            DeliverBlock();
            EncoderDrive(10, 10, .2, 2, 0);
            IMUTurn(-85, "r", .1f, 120);
            EncoderDriveSideways(-15, 25, .2, 2, -86);
            EncoderDrive(60, 40, .2, 2, -86);
        } else {
            dcmCollection.setPower(0);
            EncoderDrive(15, 40, .2, 2, -86);
            opMode.telemetry.addData("Heading: ", IMUReading);
            opMode.telemetry.update();
        }
    }
    /* Function to turn the carousel in autonomous */
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
    /* Function to deliver the freight in autonomous */
    public void DeliverBlock() {                                                                    // Deliver Block
        // Reset the lift's home position
        dcmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // If we should deliver into the high goal
        if (autonomousTargetLevel == AutonomousTargetLevel.HIGH) {
            // Raise the lift.
            dcmLift.setPower(1);
            while (dcmLift.getCurrentPosition() < 2150 && ((LinearOpMode)opMode).opModeIsActive()) {
                tmrGeneric.seconds();
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
                tmrGeneric.seconds();
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
                tmrGeneric.seconds();
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
    /* This is where you give every motor, servo, etc. a name as well a few other things */
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
        this.crsCapping = opMode.hardwareMap.get(CRServo.class, "cappingS");
        csCollection = opMode.hardwareMap.
                get(NormalizedColorSensor.class, "collectionColorSensor");
        daLights = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
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
    /* Update the color sensor. It used to be beneficial to have this be its own function,
    but it really doesn't need to be now */
    public void UpdateColorSensor() {
        collectionDistanceSensorReading = ((DistanceSensor) csCollection).getDistance(DistanceUnit.INCH);
    }
    /* Display stuff on the driver station */
    public void Telemetry() {                                                                       // Telemetry
        opMode.telemetry.addData("Drivetrain speed: ", drvTrnSpd);
        opMode.telemetry.addData("Capping speed: ", dCappingSpeed);
        opMode.telemetry.update();
    }
    /* Choose the robot's auto path */
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
        } else if (opMode.gamepad1.dpad_up) {
            alliance = Alliance.BLUE2_NO_BLOCK;
        } else if (opMode.gamepad1.dpad_down) {
            alliance = Alliance.RED2_NO_BLOCK;
        }
        // Telemeter the position currently selected
        opMode.telemetry.addData("Starting auto position", alliance);
        opMode.telemetry.update();
    }
    /* Reset the drivetrain's motor encoders */
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
    /* Play the autonomous based on what path was chosen */
    public void Autonomous() {
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
    /* Resets the imu heading by subtracting its current heading from its future headings */
    public void ResetHeading(){
        if (opMode.gamepad1.b){
            angles = imu.getAngularOrientation
                    (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            IMUReading = (angles.firstAngle);
            startingHeadingInRadians = (Math.toRadians(IMUReading) + 1.5708);
        }
    }
    /* Recall the the shipping element data from the autonomous class */
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
    // https://www.youtube.com/watch?v=HPk-VhRjNI8&list=PL3KnTfyhrIlcudeMemKd6rZFGDWyK23vx&index=1
}