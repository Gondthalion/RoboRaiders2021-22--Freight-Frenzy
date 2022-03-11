package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Maximus Prime Tele", group = "Iterative Opmode")
public class MaximusPrimeTele extends OpMode {
    MaximusPrimeBase base;
    boolean timerRestarted = false;
    @Override
    public void init() {
        base = new MaximusPrimeBase(this);
    }
    public void init_loop() {
        base.AllianceDetermination();
    }
    @Override
    public void loop() {
        if (!timerRestarted) {
            base.tmrTeleop.reset();
            timerRestarted = true;
            if (base.alliance == MaximusPrimeBase.Alliance.BLUE1) {
                base.startingHeadingInRadians =  3.14159;
            } else if (base.alliance == MaximusPrimeBase.Alliance.BLUE2) {
                base.startingHeadingInRadians = 0;
            } else if (base.alliance == MaximusPrimeBase.Alliance.RED1) {
                base.startingHeadingInRadians = 1.5708;
            } else if (base.alliance == MaximusPrimeBase.Alliance.RED2) {
                base.startingHeadingInRadians = 0;
            }
        }
        base.DriverControls();
        base.OperatorControls();
        base.UpdateDriveTrain();
        base.Telemetry();
        base.ResetHeading();
        base.Lights();
    }
    @Override
    public void stop() {
        base.stopDrivetrain();
    }
}