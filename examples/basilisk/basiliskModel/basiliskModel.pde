import com.krab.lazy.*;

void setup() {
  size(800, 800, P2D);
  basilisk = new Basilisk(new PVector(0, 0), 0.25, 0, 0.25);
  
  mono = createFont("font/RobotoMono-Regular.ttf", 128);
  textFont(mono);
  gui = new LazyGui(this);
}

void draw() {
  // Erase all.
  background(230);

  // Set coordinate system.
  translate(width / 2, height / 2);
  scale(1, -1);

  // Draw xy-axes.
  stroke(0);
  strokeWeight(1);
  line(-width / 2, 0, width / 2, 0);
  line(0, -height / 2, 0, height / 2);
  // +x label.
  pushMatrix();
  translate(width / 2, 0);
  scale(1, -1);
  fill(0);
  textSize(20);
  textAlign(RIGHT);
  text("+x", -5, -5);
  popMatrix();
  // -x label.
  pushMatrix();
  translate(-width / 2, 0);
  scale(1, -1);
  textAlign(LEFT);
  text("-x", 5, -5);
  popMatrix();
  // +y label.
  pushMatrix();
  translate(0, height / 2);
  scale(1, -1);
  text("+y", 5, 15);
  popMatrix();
  // -y label.
  pushMatrix();
  translate(0, -height / 2);
  scale(1, -1);
  text("-y", 5, -10);
  popMatrix();


  // Run Basilisk Model.
  runGui();
  basilisk.display();
}
