#!/usr/bin/env python3
"""Check the production intake command with simple gamepad and motor substitutes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
JAVA = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
STUBS = {
    'com/arcrobotics/ftclib/command/CommandBase.java': '''
      package com.arcrobotics.ftclib.command;
      public class CommandBase {
        public void addRequirements(Object... s) {} public void initialize() {}
        public void execute() {} public void end(boolean interrupted) {}
      }''',
    'com/qualcomm/robotcore/hardware/Gamepad.java': '''
      package com.qualcomm.robotcore.hardware;
      public class Gamepad {
        public float left_trigger, right_trigger; public boolean left_bumper, dpad_down;
      }''',
    'org/firstinspires/ftc/teamcode/shoot/ShootConstants.java': '''
      package org.firstinspires.ftc.teamcode.shoot;
      public class ShootConstants {
        public static final double TRIGGER_THRESHOLD=.05, INTAKE_JAM_POWER=-1, INTAKE_ACTIVE_POWER=1, TRANSIT_FEED_POWER=1;
      }''',
    'org/firstinspires/ftc/teamcode/subsystem/ShooterSubsystem.java': '''
      package org.firstinspires.ftc.teamcode.subsystem;
      public class ShooterSubsystem { public static double SHOT_TRANSIT_POWER=.8; }''',
    'org/firstinspires/ftc/teamcode/subsystem/IntakeSubsystem.java': '''
      package org.firstinspires.ftc.teamcode.subsystem;
      public class IntakeSubsystem {
        public double intake, transit;
        public void setPowers(double a,double b) { intake=a;transit=b; }
        public void stop() { setPowers(0,0); }
      }''',
    'IntakeToggleCheck.java': '''
      import com.qualcomm.robotcore.hardware.Gamepad;
      import org.firstinspires.ftc.teamcode.command.IntakeCommand;
      import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
      public class IntakeToggleCheck {
        static void check(boolean b,String m){if(!b)throw new AssertionError(m);}
        static void output(IntakeSubsystem i,double a,double b,String m){
          check(Math.abs(i.intake-a)<1e-9 && Math.abs(i.transit-b)<1e-9,m);
        }
        public static void main(String[] args) {
          Gamepad g=new Gamepad(); IntakeSubsystem i=new IntakeSubsystem(); boolean[] gate={false};
          IntakeCommand cmd=new IntakeCommand(i,g,()->gate[0]);
          cmd.initialize(); cmd.execute(); output(i,.7,0,"idle runs only intake");
          g.left_trigger=1; cmd.initialize(); cmd.execute(); output(i,1,1,"held at start runs");
          cmd.execute(); output(i,1,1,"hold keeps running");
          g.left_trigger=0; cmd.execute(); output(i,.7,0,"release returns to idle");
          g.left_trigger=1; cmd.execute(); output(i,1,1,"press resumes");
          g.left_bumper=true; cmd.execute(); output(i,-1,0,"reverse overrides");
          g.left_bumper=false; cmd.execute(); output(i,1,1,"resume after reverse");
          g.right_trigger=1; cmd.execute(); output(i,0,0,"both motors wait for blocker and delay");
          g.dpad_down=true; cmd.execute(); output(i,0,0,"RT wait overrides reverse");
          gate[0]=true; cmd.execute(); output(i,1,.8,"both motors start together");
          g.dpad_down=false;
          g.right_trigger=0; cmd.execute(); output(i,1,1,"release RT resumes LT feed while LT stays held");
          g.left_trigger=0; cmd.execute(); output(i,.7,0,"release returns to idle after shooting");
          g.dpad_down=true; cmd.execute(); output(i,-1,0,"dpad overrides idle");
          g.dpad_down=false; cmd.execute(); output(i,.7,0,"release dpad returns to idle");
          g.left_trigger=1; cmd.execute(); output(i,1,1,"press runs again");
          cmd.end(true); output(i,0,0,"interruption stops");
          check(!cmd.isIntakeEnabled(),"interruption clears state");
          System.out.println("PASS: intake idle, hold-to-run, overrides, shooting and stop");
        }
      }''',
}

with tempfile.TemporaryDirectory(prefix='intake-toggle-') as temp:
    root = Path(temp)
    for name, content in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    target = root / 'org/firstinspires/ftc/teamcode/command/IntakeCommand.java'
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(JAVA / 'command/IntakeCommand.java', target)
    java_home = Path(os.environ.get('JAVA_HOME', ''))
    javac = str(java_home / 'bin/javac') if (java_home / 'bin/javac').exists() else 'javac'
    java = str(java_home / 'bin/java') if (java_home / 'bin/java').exists() else 'java'
    subprocess.run([javac, '-encoding', 'UTF-8', '-d', str(root),
                    *map(str, root.rglob('*.java'))], check=True)
    subprocess.run([java, '-cp', str(root), 'IntakeToggleCheck'], check=True)
