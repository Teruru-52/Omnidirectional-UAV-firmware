import processing.serial.*;
Serial serial;

String string_q0;
String string_q1;
String string_q2;
String string_q3;
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;

boolean drawValues  = false;

void setup() {
  //size(displayWidth, displayHeight, OPENGL);  
   size(600, 600, P3D);
   lights();
  println(Serial.list());
  serial = new Serial(this, Serial.list()[0], 115200);
  serial.bufferUntil('\n'); // Buffer until line feed
  
  draw();
}

void draw() {
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    drawGraph();
    //saveFrame("frames/######.png");
  }
}

void drawGraph() {
  background(230);
  //camera(0, 1000, 1000, 0, 0, 0, 0, 1, 0);
  translate(width / 2, height / 2, 0);
  //translate(200, 180);
  
  q0 = float(string_q0);
  q1 = float(string_q1);
  q2 = float(string_q2);
  q3 = float(string_q3);
  
  stroke(0);
  fill(0);
  textSize(25);
  text("q0 = " + string_q0, 160, 80); 
  text("q1 = " + string_q1, 160, 120); 
  text("q2 = " + string_q2, 160,160);
  text("q3 = " + string_q3, 160,200);
  
  fill(255);
  pushMatrix(); // begin object
  applyMatrix(q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2), 0,
              2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3-q0*q1), 0,
              2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0*q0-q1*q1-q2*q2+q3*q3, 0,
              0, 0, 0, 1);
  int size = 10;
  box(20 * size, 15 * size, 1 * size);
  //translate(0, -4 * size, 0);
  popMatrix(); // end of object 
}

void serialEvent(Serial serial) { 
  string_q0 = serial.readStringUntil('\t');
  string_q1 = serial.readStringUntil('\t');
  string_q2 = serial.readStringUntil('\t'); 
  string_q3 = serial.readStringUntil('\t'); 
  
  serial.clear(); // Clear buffer
  drawValues = true;
}
