/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */
public class CompetitionOpMode extends LinearOpMode {



  DcMotor LeftFrontMotor;
  DcMotor LeftBackMotor;
  DcMotor RightFrontMotor;
  DcMotor RightBackMotor;

  @Override
  public void runOpMode() throws InterruptedException {
      LeftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
      LeftBackMotor = hardwareMap.dcMotor.get("left_back_drive");
      RightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
      RightBackMotor = hardwareMap.dcMotor.get("right_back_drive");


      LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
      LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);

      sleep(20);
      //LeftFrontMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
      //LeftBackMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
      //RightFrontMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
      //RightBackMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

      LeftFrontMotor.setMode
              (DcMotor.RunMode.STOP_AND_RESET_ENCODER
              );
      LeftBackMotor.setMode
              (DcMotor.RunMode.STOP_AND_RESET_ENCODER
              );
      RightFrontMotor.setMode
              (DcMotor.RunMode.STOP_AND_RESET_ENCODER
              );
      RightBackMotor.setMode
              (DcMotor.RunMode.STOP_AND_RESET_ENCODER
              );


      telemetry.addData("LF: ", LeftFrontMotor.getCurrentPosition());
      telemetry.addData("LB: ", LeftBackMotor.getCurrentPosition());
      telemetry.addData("RF: ", RightFrontMotor.getCurrentPosition());
      telemetry.addData("RB: ", RightBackMotor.getCurrentPosition());
      waitForStart();


          /*LeftFrontMotor.setTargetPosition(0); //1680 ppr 21000pulses target
          LeftBackMotor.setTargetPosition(0);
          RightFrontMotor.setTargetPosition(0);
          RightBackMotor.setTargetPosition(0);
        LeftFrontMotor.setTargetPosition(21000); //1680 ppr 21000pulses target
          LeftBackMotor.setTargetPosition(21000);
        RightFrontMotor.setTargetPosition(21000);
        RightBackMotor.setTargetPosition(21000);*/
      LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      int left_count = 9200;
      int right_count = 15200;
      LeftFrontMotor.setPower(1);
      LeftBackMotor.setPower(1);
      RightFrontMotor.setPower(1);
      RightBackMotor.setPower(1);
      while ((Math.abs(RightFrontMotor.getCurrentPosition()) < right_count)) {
          telemetry.addData("LF: ", LeftFrontMotor.getCurrentPosition());
          telemetry.addData("LB: ", LeftBackMotor.getCurrentPosition());
          telemetry.addData("RF: ", RightFrontMotor.getCurrentPosition());
          telemetry.addData("RB: ", RightBackMotor.getCurrentPosition());
          telemetry.addData("AVG: ", (LeftFrontMotor.getCurrentPosition() + LeftBackMotor.getCurrentPosition()
                  + RightFrontMotor.getCurrentPosition() + RightBackMotor.getCurrentPosition())/4);
          if (Math.abs(LeftFrontMotor.getCurrentPosition()) > left_count) {
              LeftFrontMotor.setPower(-1.0);
              LeftBackMotor.setPower(-1.0);
          }
      }


      LeftFrontMotor.setPower(0);
      LeftBackMotor.setPower(0);
      RightFrontMotor.setPower(0);
      RightBackMotor.setPower(0);
      LeftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      LeftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      RightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      //sleep(20);

      //sleep(2000);


       /*while (opModeIsActive()) {
          LeftFrontMotor.setPower(Range.clip(-gamepad1.left_stick_y, -1, 1));
          LeftBackMotor.setPower(Range.clip(-gamepad1.left_stick_y, -1, 1));
          RightFrontMotor.setPower(Range.clip(-gamepad1.right_stick_y, -1, 1));
          RightBackMotor.setPower(Range.clip(-gamepad1.right_stick_y, -1, 1));
          telemetry.addData("LF: ", LeftFrontMotor.getConnectionInfo());
          telemetry.addData("LB: ", LeftBackMotor.getConnectionInfo());
          telemetry.addData("RF: ", RightFrontMotor.getConnectionInfo());
          telemetry.addData("RB: ", RightBackMotor.getConnectionInfo());
          waitOneFullHardwareCycle();
        }*/
  }

}
