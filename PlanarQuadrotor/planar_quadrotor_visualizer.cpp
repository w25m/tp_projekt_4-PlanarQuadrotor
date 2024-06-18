#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void rotate_point(int cx, int cy, double angle, int& x, int& y) {
    double s = sin(angle);
    double c = cos(angle);
x -= cx;
y -= cy;

// Rotate point
int xnew = static_cast<int>(x * c - y * s);
int ynew = static_cast<int>(x * s + y * c);

// Translate point back
x = xnew + cx;
y = ynew + cy;
}

void draw_filled_rect(SDL_Renderer* renderer, SDL_Point* points, int n, SDL_Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (int i = 0; i < n; ++i) {
        SDL_RenderDrawLine(renderer, points[i].x, points[i].y, points[(i + 1) % n].x, points[(i + 1) % n].y);
    }
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state_f = quadrotor_ptr->GetState().cast<float>();
    int q_x = static_cast<int>(state_f[0]);
    int q_y = static_cast<int>(state_f[1]);
    double q_theta = state_f[2];
// Calculate tilt based on horizontal velocity (state_f[3])
double tilt_angle = state_f[3] * 0.000001; // Adjust the multiplier as necessary for desired tilt effect

// Combine the base orientation and the tilt angle
double total_angle = q_theta + tilt_angle;

// Set line color and blend mode
SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
SDL_SetRenderDrawBlendMode(gRenderer.get(), SDL_BLENDMODE_BLEND);

// Draw wings
int x1 = q_x - 80, y1 = q_y - 10;
int x2 = q_x + 80, y2 = q_y - 10;
rotate_point(q_x, q_y, total_angle, x1, y1);
rotate_point(q_x, q_y, total_angle, x2, y2);
SDL_RenderDrawLine(gRenderer.get(), x1, y1, x2, y2); // top wing

x1 = q_x - 80, y1 = q_y + 10;
x2 = q_x + 80, y2 = q_y + 10;
rotate_point(q_x, q_y, total_angle, x1, y1);
rotate_point(q_x, q_y, total_angle, x2, y2);
SDL_RenderDrawLine(gRenderer.get(), x1, y1, x2, y2); // bottom wing

// Draw center body
SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);

SDL_Point body[4] = {
    {q_x - 80, q_y - 10},
    {q_x + 80, q_y - 10},
    {q_x + 80, q_y + 10},
    {q_x - 80, q_y + 10}
};
SDL_Point leftRect[4] = {
    {q_x - 70, q_y - 60},
    {q_x - 60, q_y - 60},
    {q_x - 60, q_y},
    {q_x - 70, q_y}
};
SDL_Point rightRect[4] = {
    {q_x + 60, q_y - 60},
    {q_x + 70, q_y - 60},
    {q_x + 70, q_y},
    {q_x + 60, q_y}
};

// Rotate the points
for (int i = 0; i < 4; ++i) {
    rotate_point(q_x, q_y, q_theta, body[i].x, body[i].y);
    rotate_point(q_x, q_y, q_theta, leftRect[i].x, leftRect[i].y);
    rotate_point(q_x, q_y, q_theta, rightRect[i].x, rightRect[i].y);
}

// Draw the rectangles
SDL_Color blue = { 0x00, 0x00, 0xFF, 0xFF };
SDL_Color red = { 0xFF, 0x00, 0x00, 0xFF };
draw_filled_rect(gRenderer.get(), body, 4, blue);
draw_filled_rect(gRenderer.get(), leftRect, 4, red);
draw_filled_rect(gRenderer.get(), rightRect, 4, red);

// Left wing
SDL_Point left_lWing[6] = {
    { q_x - 70, q_y - 40 },
    { q_x - 90, q_y - 45 },
    { q_x - 100, q_y - 42 },
    { q_x - 100, q_y - 40 },
    { q_x - 90, q_y - 30 },
    { q_x - 70, q_y - 35 }
};
for (int i = 0; i < 6; ++i) {
    int x = left_lWing[i].x, y = left_lWing[i].y;
    rotate_point(q_x, q_y, total_angle, x, y);
    left_lWing[i].x = x;
    left_lWing[i].y = y;
}
SDL_RenderDrawLines(gRenderer.get(), left_lWing, 6);

// Right wing
SDL_Point right_rWing[6] = {
    { q_x + 70, q_y - 40 },
    { q_x + 90, q_y - 45 },
    { q_x + 100, q_y - 42 },
    { q_x + 100, q_y - 40 },
    { q_x + 90, q_y - 30 },
    { q_x + 70, q_y - 35 }
};
for (int i = 0; i < 6; ++i) {
    int x = right_rWing[i].x, y = right_rWing[i].y;
    rotate_point(q_x, q_y, total_angle, x, y);
    right_rWing[i].x = x;
    right_rWing[i].y = y;
}
SDL_RenderDrawLines(gRenderer.get(), right_rWing, 6);

// Left wing (bottom)
SDL_Point left_rWing[6] = {
    { q_x - 60, q_y - 40 },
    { q_x - 40, q_y - 45 },
    { q_x - 30, q_y - 42 },
    { q_x - 30, q_y - 40 },
    { q_x - 40, q_y - 30 },
    { q_x - 60, q_y - 35 }
};
for (int i = 0; i < 6; ++i) {
    int x = left_rWing[i].x, y = left_rWing[i].y;
    rotate_point(q_x, q_y, total_angle, x, y);
    left_rWing[i].x = x;
    left_rWing[i].y = y;
}
SDL_RenderDrawLines(gRenderer.get(), left_rWing, 6);

// Right wing (bottom)
SDL_Point right_lWing[6] = {
    { q_x + 60, q_y - 40 },
    { q_x + 40, q_y - 45 },
    { q_x + 30, q_y - 42 },
    { q_x + 30, q_y - 40 },
    { q_x + 40, q_y - 30 },
    { q_x + 60, q_y - 35 }
};
for (int i = 0; i < 6; ++i) {
    int x = right_lWing[i].x, y = right_lWing[i].y;
    rotate_point(q_x, q_y, total_angle, x, y);
    right_lWing[i].x = x;
    right_lWing[i].y = y;
}
SDL_RenderDrawLines(gRenderer.get(), right_lWing, 6);
}
