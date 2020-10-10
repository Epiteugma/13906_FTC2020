package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Test", group = "FTC_Cyprus_2020-21")
public class autonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor fl = hardwareMap.get(DcMotor.class, "front_left");
        final DcMotor fr = hardwareMap.get(DcMotor.class, "front_right");
        final DcMotor bl = hardwareMap.get(DcMotor.class, "back_left");
        final DcMotor br = hardwareMap.get(DcMotor.class, "back_right");

        waitForStart();
        Runnable test = new Runnable() {
            @Override
            public void run() {
                fl.setPower(0.5);
                fr.setPower(-0.5);
                bl.setPower(0.5);
                br.setPower(-0.5);
                try {Thread.sleep(1000);}
                catch (Exception e) {}
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }
        };
        Thread thread1 = new Thread(test);
        thread1.start();
        while(opModeIsActive()) {

        }
    }

}
