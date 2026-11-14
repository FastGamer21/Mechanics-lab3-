#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

float PROJ_SPEED = 2000.0f;
float RESTITUTION = 0.85f;
float FAST_FREQ = 240.0f;

#define BASE_FREQ 60.0f
#define GRAVITY 400.0f
#define W 1200
#define H 700
#define WALL_THICK 10.0f

int TYPE_STATIC = 0;
int TYPE_NORMAL = 1;
int TYPE_FAST = 2;

struct RectObj {
    float x, y;
    float w, h;
    float vx, vy;
    float mass;
    float invMass;
    int type;
    sf::Color color;
};

bool check_overlap(RectObj& a, RectObj& b) {
    float a_left = a.x - a.w / 2;
    float a_right = a.x + a.w / 2;
    float a_top = a.y - a.h / 2;
    float a_bottom = a.y + a.h / 2;

    float b_left = b.x - b.w / 2;
    float b_right = b.x + b.w / 2;
    float b_top = b.y - b.h / 2;
    float b_bottom = b.y + b.h / 2;

    return (a_left < b_right && a_right > b_left &&
        a_top < b_bottom && a_bottom > b_top);
}

// Õûõû))) Äåëàåò ïðûã-ïðûã)))
void resolve_collision(RectObj& a, RectObj& b) {
    float dx = a.x - b.x;
    float px = (a.w + b.w) / 2.0f - std::abs(dx);
    if (px <= 0) return;

    float dy = a.y - b.y;
    float py = (a.h + b.h) / 2.0f - std::abs(dy);
    if (py <= 0) return;

    float nx = 0, ny = 0, penetration = 0;
    if (px < py) {
        if (dx > 0) nx = 1; else nx = -1;
        penetration = px;
    }
    else {
        if (dy > 0) ny = 1; else ny = -1;
        penetration = py;
    }

    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass == 0) return;

    float corr = std::max(penetration - 0.05f, 0.0f) / totalInvMass * 0.8f;
    a.x += nx * corr * a.invMass;
    a.y += ny * corr * a.invMass;
    b.x -= nx * corr * b.invMass;
    b.y -= ny * corr * b.invMass;

    float rvx = a.vx - b.vx;
    float rvy = a.vy - b.vy;
    float velAlongNormal = rvx * nx + rvy * ny;

    if (velAlongNormal > 0) return;

    float e = RESTITUTION;
    if (std::abs(velAlongNormal) < 25.0f) e = 0.0f;

    float j = -(1.0f + e) * velAlongNormal / totalInvMass;

    a.vx += j * nx * a.invMass;
    a.vy += j * ny * a.invMass;
    b.vx -= j * nx * b.invMass;
    b.y -= j * ny * b.invMass;
}

// ÑCD (ß ×¨ ÏÈØÓ ÑÅÁÅ ÑÑÄøíèê ÄËß ÏÊ????)
bool swept_aabb(RectObj& moving, RectObj& target, float dt, float& outTime, float& nx, float& ny) {
    float rvx = (moving.vx - target.vx) * dt;
    float rvy = (moving.vy - target.vy) * dt;

    float minX = (target.x - target.w / 2) - moving.w / 2;
    float maxX = (target.x + target.w / 2) + moving.w / 2;
    float minY = (target.y - target.h / 2) - moving.h / 2;
    float maxY = (target.y + target.h / 2) + moving.h / 2;

    float txEntry, txExit, tyEntry, tyExit;

    if (rvx == 0.0f) {
        if (moving.x <= minX || moving.x >= maxX) return false;
        txEntry = -999999.0f;
        txExit = 999999.0f;
    }
    else {
        txEntry = std::min((minX - moving.x) / rvx, (maxX - moving.x) / rvx);
        txExit = std::max((minX - moving.x) / rvx, (maxX - moving.x) / rvx);
    }

    if (rvy == 0.0f) {
        if (moving.y <= minY || moving.y >= maxY) return false;
        tyEntry = -999999.0f;
        tyExit = 999999.0f;
    }
    else {
        tyEntry = std::min((minY - moving.y) / rvy, (maxY - moving.y) / rvy);
        tyExit = std::max((minY - moving.y) / rvy, (maxY - moving.y) / rvy);
    }

    float tEntry = std::max(txEntry, tyEntry);
    float tExit = std::min(txExit, tyExit);

    if (tEntry > tExit || tExit < 0.0f || tEntry < 0.0f || tEntry > 1.0f) {
        return false;
    }

    if (txEntry > tyEntry) {
        if (rvx > 0) nx = -1; else nx = 1;
        ny = 0;
    }
    else {
        nx = 0;
        if (rvy > 0) ny = -1; else ny = 1;
    }

    outTime = tEntry * dt;
    return true;
}

void init_scene(std::vector<RectObj>& objs) {
    objs.clear();
    objs.push_back({ W / 2.0f, H - 20.0f, W, 40, 0, 0, 0, 0, TYPE_STATIC, sf::Color(90, 90, 100) });
    objs.push_back({ W / 2.0f, 20.0f, W, 40, 0, 0, 0, 0, TYPE_STATIC, sf::Color(90, 90, 100) });
    objs.push_back({ 20.0f, H / 2.0f, 40, H, 0, 0, 0, 0, TYPE_STATIC, sf::Color(90, 90, 100) });
    objs.push_back({ W - 20.0f, H / 2.0f, 40, H, 0, 0, 0, 0, TYPE_STATIC, sf::Color(90, 90, 100) });
    objs.push_back({ 750.0f, 260.0f, WALL_THICK, 360, 0, 0, 0, 0, TYPE_STATIC, sf::Color(255, 80, 80) });

    for (int i = 0; i < 4; i++) {
        objs.push_back({ 350.0f + i * 160.0f, 500.0f, 48.0f, 48.0f, 0, 0, 0, 0, TYPE_STATIC, sf::Color(120, 120, 140) });
    }

    for (int i = 0; i < 6; i++) {
        objs.push_back({ 950.0f, H - 40.0f - 12.0f - i * 24.0f, 36.0f, 24.0f, 0, 0, 2.0f, 0.5f, TYPE_NORMAL, sf::Color(100, 220, 130) });
    }

    for (int i = 0; i < 14; i++) {
        float size = 24.0f * (0.8f + (i % 3) * 0.3f);
        objs.push_back({ 120.0f + (i % 7) * 75.0f, 100.0f + (i / 7) * 60.0f, size, size,
                        80.0f - (i % 5) * 35.0f, 20.0f * (i % 3), 1.0f, 1.0f, TYPE_NORMAL, sf::Color(90, 170, 255) });
    }
    objs.push_back({ 100.0f, 220.0f, 12.0f, 12.0f, PROJ_SPEED, 0.0f, 0.5f, 2.0f, TYPE_FAST, sf::Color(255, 220, 50) });
    objs.push_back({ 100.0f, 310.0f, 12.0f, 12.0f, PROJ_SPEED * 0.9f, -120.0f, 0.5f, 2.0f, TYPE_FAST, sf::Color(255, 180, 50) });
    objs.push_back({ 100.0f, 140.0f, 12.0f, 12.0f, PROJ_SPEED * 1.1f, 150.0f, 0.5f, 2.0f, TYPE_FAST, sf::Color(255, 140, 50) });
}

// Ôèçåêà))
void step_discrete(std::vector<RectObj>& objs, float dt) {
    for (int i = 0; i < objs.size(); i++) {
        if (objs[i].type != TYPE_STATIC) {
            if (objs[i].type == TYPE_NORMAL) objs[i].vy += GRAVITY * dt;
            objs[i].x += objs[i].vx * dt;
            objs[i].y += objs[i].vy * dt;
        }
    }

    for (int i = 0; i < objs.size(); i++) {
        for (int j = i + 1; j < objs.size(); j++) {
            if (objs[i].type == TYPE_STATIC && objs[j].type == TYPE_STATIC) continue;
            if (check_overlap(objs[i], objs[j])) {
                resolve_collision(objs[i], objs[j]);
            }
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(W, H), "Lab 3 Mechanics");
    window.setFramerateLimit(60);

    sf::Font font;
    font.loadFromFile("C:/Windows/Fonts/consola.ttf");

    std::vector<RectObj> objects;
    init_scene(objects);

    int mode = 2;
    bool paused = false;
    bool next_frame = false;
    float dt60 = 1.0f / BASE_FREQ;
    float accum = 0.0f;
    float time_history[60] = { 0 };
    int time_idx = 0;
    int sub_steps = 1;

    sf::Clock loop_clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Num1) { mode = 1; init_scene(objects); }
                if (event.key.code == sf::Keyboard::Num2) { mode = 2; init_scene(objects); }
                if (event.key.code == sf::Keyboard::Num3) { mode = 3; accum = 0; init_scene(objects); }
                if (event.key.code == sf::Keyboard::Num4) { mode = 4; init_scene(objects); }
                if (event.key.code == sf::Keyboard::R) { accum = 0; init_scene(objects); }
                if (event.key.code == sf::Keyboard::P) { paused = !paused; }
                if (event.key.code == sf::Keyboard::N) { next_frame = true; }
                if (event.key.code == sf::Keyboard::Space) {
                    objects.push_back({ 100.0f, 220.0f, 12.0f, 12.0f, PROJ_SPEED, 0.0f, 0.5f, 2.0f, TYPE_FAST, sf::Color(255, 220, 50) });
                }
            }
        }

        if (!paused || next_frame) {
            next_frame = false;

            sf::Clock step_timer;

            if (mode == 1) {
                sub_steps = 1;
                step_discrete(objects, dt60);
            }
            else if (mode == 2) {
                sub_steps = 1;
                for (int i = 0; i < objects.size(); i++) {
                    if (objects[i].type == TYPE_FAST) {
                        float rem_dt = dt60;
                        for (int bounce = 0; bounce < 3; bounce++) {
                            float best_t = rem_dt;
                            int hit_idx = -1;
                            float hitNx = 0, hitNy = 0;

                            for (int j = 0; j < objects.size(); j++) {
                                if (i == j) continue;
                                float tc = 0, nx = 0, ny = 0;
                                if (swept_aabb(objects[i], objects[j], rem_dt, tc, nx, ny)) {
                                    if (tc < best_t) {
                                        best_t = tc;
                                        hit_idx = j;
                                        hitNx = nx;
                                        hitNy = ny;
                                    }
                                }
                            }

                            objects[i].x += objects[i].vx * best_t;
                            objects[i].y += objects[i].vy * best_t;
                            rem_dt -= best_t;

                            if (hit_idx != -1) {
                                float totalInv = objects[i].invMass + objects[hit_idx].invMass;
                                float vn = (objects[i].vx - objects[hit_idx].vx) * hitNx + (objects[i].vy - objects[hit_idx].vy) * hitNy;
                                if (vn < 0) {
                                    float jImp = -(1.0f + RESTITUTION) * vn / totalInv;
                                    objects[i].vx += jImp * hitNx * objects[i].invMass;
                                    objects[i].vy += jImp * hitNy * objects[i].invMass;
                                    objects[hit_idx].vx -= jImp * hitNx * objects[hit_idx].invMass;
                                    objects[hit_idx].vy -= jImp * hitNy * objects[hit_idx].invMass;
                                }
                            }
                            else {
                                break;
                            }
                        }
                    }
                }

                for (int i = 0; i < objects.size(); i++) {
                    if (objects[i].type == TYPE_NORMAL) {
                        objects[i].vy += GRAVITY * dt60;
                        objects[i].x += objects[i].vx * dt60;
                        objects[i].y += objects[i].vy * dt60;
                    }
                }
                for (int i = 0; i < objects.size(); i++) {
                    for (int j = i + 1; j < objects.size(); j++) {
                        if (objects[i].type == TYPE_STATIC && objects[j].type == TYPE_STATIC) continue;
                        if (objects[i].type == TYPE_FAST || objects[j].type == TYPE_FAST) continue;
                        if (check_overlap(objects[i], objects[j])) {
                            resolve_collision(objects[i], objects[j]);
                        }
                    }
                }
            }
            else if (mode == 3) {
                float dtFast = 1.0f / FAST_FREQ;
                accum += dt60;
                int steps = 0;

                while (accum >= dtFast) {
                    for (int i = 0; i < objects.size(); i++) {
                        if (objects[i].type == TYPE_FAST) {
                            objects[i].x += objects[i].vx * dtFast;
                            objects[i].y += objects[i].vy * dtFast;
                        }
                    }
                    for (int i = 0; i < objects.size(); i++) {
                        if (objects[i].type != TYPE_FAST) continue;
                        for (int j = 0; j < objects.size(); j++) {
                            if (i != j && check_overlap(objects[i], objects[j])) {
                                resolve_collision(objects[i], objects[j]);
                            }
                        }
                    }
                    accum -= dtFast;
                    steps++;
                }
                sub_steps = steps;

                for (int i = 0; i < objects.size(); i++) {
                    if (objects[i].type == TYPE_NORMAL) {
                        objects[i].vy += GRAVITY * dt60;
                        objects[i].x += objects[i].vx * dt60;
                        objects[i].y += objects[i].vy * dt60;
                    }
                }
                for (int i = 0; i < objects.size(); i++) {
                    for (int j = i + 1; j < objects.size(); j++) {
                        if (objects[i].type == TYPE_FAST || objects[j].type == TYPE_FAST) continue;
                        if (objects[i].type == TYPE_STATIC && objects[j].type == TYPE_STATIC) continue;
                        if (check_overlap(objects[i], objects[j])) {
                            resolve_collision(objects[i], objects[j]);
                        }
                    }
                }
            }
            else if (mode == 4) {
                float dtFast = 1.0f / FAST_FREQ;
                int steps = std::round(FAST_FREQ / BASE_FREQ);
                sub_steps = steps;
                for (int s = 0; s < steps; s++) {
                    step_discrete(objects, dtFast);
                }
            }

            time_history[time_idx] = step_timer.getElapsedTime().asMicroseconds() / 1000.0f;
            time_idx = (time_idx + 1) % 60;
        }

        float sum_time = 0;
        for (int i = 0; i < 60; i++) sum_time += time_history[i];
        float avg_time = sum_time / 60.0f;

        window.clear(sf::Color(22, 24, 30));

        for (int i = 0; i < objects.size(); i++) {
            sf::RectangleShape shape(sf::Vector2f(objects[i].w, objects[i].h));
            shape.setPosition(objects[i].x - objects[i].w / 2, objects[i].y - objects[i].h / 2);
            shape.setFillColor(objects[i].color);
            window.draw(shape);
        }

        std::string mode_str = "";
        if (mode == 1) mode_str = "DISCRETE_60";
        if (mode == 2) mode_str = "CCD";
        if (mode == 3) mode_str = "MULTI_RATE";
        if (mode == 4) mode_str = "ALL_240";
        std::string stat_str = "RUNNING";
        if (paused) stat_str = "PAUSED";
        std::string line1 = "Mode [1-4]: " + mode_str + " (" + stat_str + ") | [P] Pause [N] Step [R] Reset [SPACE] Fire";
        std::string line2 = "Physics step time: " + std::to_string(avg_time) + " ms | Fast sub-steps: " + std::to_string(sub_steps);
        std::string line3 = "v=" + std::to_string((int)PROJ_SPEED) + " px/s | wall=" + std::to_string((int)WALL_THICK) +
            " px | e=" + std::to_string(RESTITUTION) + " | FastFreq=" + std::to_string((int)FAST_FREQ) + " Hz";

        sf::Text text1(line1, font, 16); text1.setPosition(50, 48); text1.setFillColor(sf::Color::White);
        sf::Text text2(line2, font, 16); text2.setPosition(50, 70); text2.setFillColor(sf::Color::White);
        sf::Text text3(line3, font, 16); text3.setPosition(50, 92); text3.setFillColor(sf::Color::White);

        window.draw(text1);
        window.draw(text2);
        window.draw(text3);

        window.display();
    }

    return 0;
}