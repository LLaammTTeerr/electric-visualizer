#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <vector>
#include <memory>
#include <random>
#include <chrono>

const int SCREEN_WIDTH = 2000;
const int SCREEN_HEIGHT = 1200;
const int TARGET_FPS = 120;
const float DT = 1.0f / TARGET_FPS;
const float K = 8.99e9;
const float MIN_DISTANCE = 20.0f;
const int BUTTON_WIDTH = 200;
const int BUTTON_HEIGHT = 50;

std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

class Charge {
public:
  Vector2 position;
  Vector2 velocity;
  float charge;
  float mass;

  Charge(Vector2 pos, Vector2 vel, float q, float m) 
    : position(pos), velocity(vel), charge(q), mass(m) {}
  
  virtual ~Charge() = default;
  virtual void draw() const = 0;
  virtual void update(const std::vector<std::shared_ptr<Charge>>& charges) = 0;
  virtual void handleCollision(std::shared_ptr<Charge> other) = 0;
};

class PointCharge : public Charge {
public:
  PointCharge(Vector2 pos, Vector2 vel, float q, float m = 1.0f) 
    : Charge(pos, vel, q, m) {}

  void handleCollision(std::shared_ptr<Charge> other) override {
    auto otherPoint = std::dynamic_pointer_cast<PointCharge>(other);
    if (!otherPoint) return;

    Vector2 diff = Vector2Subtract(position, otherPoint->position);
    float distance = Vector2Length(diff);

    if (distance < MIN_DISTANCE) {
      Vector2 normal = Vector2Scale(diff, 1.0f/distance);
      Vector2 relativeVel = Vector2Subtract(velocity, otherPoint->velocity);
      
      float restitution = 0.8f;
      float j = -(1.0f + restitution) * Vector2DotProduct(relativeVel, normal);
      j /= 1.0f/mass + 1.0f/otherPoint->mass;
      
      Vector2 impulse = Vector2Scale(normal, j);
      velocity = Vector2Add(velocity, Vector2Scale(impulse, 1.0f/mass));
      otherPoint->velocity = Vector2Subtract(otherPoint->velocity, 
                                           Vector2Scale(impulse, 1.0f/otherPoint->mass));
      
      float overlap = MIN_DISTANCE - distance;
      Vector2 separation = Vector2Scale(normal, overlap * 0.5f);
      position = Vector2Add(position, separation);
      otherPoint->position = Vector2Subtract(otherPoint->position, separation);
    }
  }

  void update(const std::vector<std::shared_ptr<Charge>>& charges) override {
    Vector2 totalForce = {0, 0};
    
    for (const auto& other : charges) {
      if (other.get() == this) continue;
      
      Vector2 r = Vector2Subtract(position, other->position);
      float r_mag = fmaxf(Vector2Length(r), MIN_DISTANCE);
      float force_mag = K * charge * other->charge / (r_mag * r_mag);
      
      float max_force = 1e5;
      force_mag = Clamp(force_mag, -max_force, max_force);
      
      Vector2 force = Vector2Scale(Vector2Normalize(r), force_mag);
      totalForce = Vector2Add(totalForce, force);
    }

    Vector2 acceleration = Vector2Scale(totalForce, 1.0f/mass);
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, DT));
    velocity = Vector2Scale(velocity, 0.999f);
    
    float maxSpeed = 1000.0f;
    float speedSq = Vector2LengthSqr(velocity);
    if (speedSq > maxSpeed * maxSpeed) {
      velocity = Vector2Scale(Vector2Normalize(velocity), maxSpeed);
    }
    
    position = Vector2Add(position, Vector2Scale(velocity, DT));
    
    if (position.x < 0 || position.x > SCREEN_WIDTH) {
      velocity.x *= -0.8f;
      position.x = Clamp(position.x, 0, (float)SCREEN_WIDTH);
    }
    if (position.y < 0 || position.y > SCREEN_HEIGHT) {
      velocity.y *= -0.8f;
      position.y = Clamp(position.y, 0, (float)SCREEN_HEIGHT);
    }
  }

  void draw() const override {
    Color color = charge > 0 ? RED : BLUE;
    DrawCircleV(position, 10, color);
  }
};

class ElectricField {
private:
  std::vector<std::shared_ptr<Charge>> charges;
  bool isSimulationRunning = false;

public:
  void addCharge(std::shared_ptr<Charge> charge) {
    charges.push_back(charge);
  }

  void toggleSimulation() {
    isSimulationRunning = !isSimulationRunning;
  }

  bool isRunning() const {
    return isSimulationRunning;
  }

  void update() {
    if (!isSimulationRunning) return;
    
    for (auto& charge : charges) {
      charge->update(charges);
    }

    for (size_t i = 0; i < charges.size(); i++) {
      for (size_t j = i + 1; j < charges.size(); j++) {
        charges[i]->handleCollision(charges[j]);
      }
    }
  }

  void draw() const {
    for (const auto& charge : charges) {
      charge->draw();
    }
  }

  void drawStartButton() const {
    Rectangle btnBounds = {
      (float)(GetScreenWidth() - BUTTON_WIDTH) / 2,
      (float)(GetScreenHeight() - BUTTON_HEIGHT - 20),
      BUTTON_WIDTH,
      BUTTON_HEIGHT
    };

    DrawRectangleRec(btnBounds, isSimulationRunning ? GREEN : RED);
    DrawRectangleLinesEx(btnBounds, 2, WHITE);
    
    const char* text = isSimulationRunning ? "STOP" : "START";
    int fontSize = 30;
    Vector2 textSize = MeasureTextEx(GetFontDefault(), text, fontSize, 1);
    Vector2 textPos = {
      btnBounds.x + (btnBounds.width - textSize.x) / 2,
      btnBounds.y + (btnBounds.height - textSize.y) / 2
    };
    
    DrawText(text, textPos.x, textPos.y, fontSize, WHITE);

    Vector2 mousePoint = GetMousePosition();
    if (CheckCollisionPointRec(mousePoint, btnBounds)) {
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        const_cast<ElectricField*>(this)->toggleSimulation();
      }
    }
  }
};

Vector2 randomPosition() {
  return {
    (float)GetRandomValue(0, SCREEN_WIDTH),
    (float)GetRandomValue(0, SCREEN_HEIGHT)
  };
}

int main() {
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Electric Field Simulator");
  SetTargetFPS(TARGET_FPS);

  ElectricField field;
  
  for (int i = 0; i < 2; i++) {
    Vector2 pos = randomPosition();
    Vector2 vel = {0, 0};
    float charge = i ? -1 : +1;
    field.addCharge(std::make_shared<PointCharge>(pos, vel, charge, 1.0f));
  } 

  while (!WindowShouldClose()) {
    field.update();
    BeginDrawing();
    ClearBackground(BLACK);
    field.draw();
    field.drawStartButton();
    EndDrawing();
  }

  CloseWindow();
  return 0;
}
