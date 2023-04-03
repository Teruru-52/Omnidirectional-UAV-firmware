import processing.serial.*;
Serial serial;

String stringRoll;
String stringPitch;
String stringYaw;
float roll = 0;
float pitch = 0;
float yaw = 0;

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
  
  roll = float(stringRoll);
  pitch = float(stringPitch);
  yaw = float(stringYaw);
  
  stroke(0);
  fill(0);
  textSize(25);
  text("roll = " + stringRoll, 160, 80); 
  text("pitch = " + stringPitch, 160, 120); 
  text("yaw = " + stringYaw, 160,160);
  
  fill(255);
  rotateZ(yaw);
  rotateY(pitch);
  rotateX(roll);
  
  int size = 10;
  box(20 * size, 15 * size, 1 * size);
  translate(0, -4 * size, 0);
 
}

void serialEvent(Serial serial) { 
  stringRoll = serial.readStringUntil('\t');
  stringPitch = serial.readStringUntil('\t');
  stringYaw = serial.readStringUntil('\t'); 
  
  serial.clear(); // Clear buffer
  drawValues = true;
}
