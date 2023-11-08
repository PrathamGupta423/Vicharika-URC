import processing.serial.*;

import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import cc.arduino.*;
import org.firmata.*;

ControlDevice cont;
ControlIO control;

//Arduino arduino;

float R_JS_Horizontal;
float R_JS_Vertical;
float L_JS_Horizontal;
float L_JS_Vertical;


void setup() {
  size(360, 200);
  
  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("ARM_C");
  
   if (cont == null) {
    println("not today chump");
    System.exit(-1);
  }
  
  ////println(Arduino.list());
  //arduino = new Arduino(this, Arduino.list()[2], 57600);
  //arduino.pinMode(10, Arduino.SERVO);
  
}

public void getUserInput(){
 // assign our float value 
 //access the controller.
 R_JS_Horizontal = map(cont.getSlider("R_JS_Horizontal").getValue(), -1, 1, -256, 256);
 R_JS_Vertical = map(cont.getSlider("R_JS_Vertical").getValue(), -1, 1, -256, 256);
 L_JS_Horizontal = map(cont.getSlider("L_JS_Horizontal").getValue(), -1, 1, -256, 256);
 L_JS_Vertical = map(cont.getSlider("L_JS_Vertical").getValue(), -1, 1, -256, 256);
 
 println(R_JS_Horizontal ,R_JS_Vertical ,L_JS_Horizontal ,L_JS_Vertical);

 
}

void draw(){
 getUserInput();
 //background(thumb,100,255);
 
 //arduino.servoWrite(10, (int)thumb);
}
