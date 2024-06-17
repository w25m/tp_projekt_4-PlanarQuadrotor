#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void rotate_point(int cx, int cy, int angle, int& x, int& y) {
    int s = sin(static_cast<double>(angle));
    int c = cos(static_cast<double>(angle));

    // Translate point back to origin
    x -= cx;
    y -= cy;

    // Rotate point
    int xnew = x * c - y * s;
    int ynew = x * s + y * c;

    // Translate point back
    x = xnew + cx;
    y = ynew + cy;
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state_f = quadrotor_ptr->GetState().cast<float>();
    int q_x, q_y, q_theta;

    q_x = static_cast<int>(state_f[0]); // Rzutowanie na int
    q_y = static_cast<int>(state_f[1]); // Rzutowanie na int
    q_theta = static_cast<int>(state_f[2]); // Rzutowanie na int

    // Set line color and blend mode
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_SetRenderDrawBlendMode(gRenderer.get(), SDL_BLENDMODE_BLEND);

    // Draw wings
    int x1 = q_x - 80, y1 = q_y - 10;
    int x2 = q_x + 80, y2 = q_y - 10;
    rotate_point(q_x, q_y, q_theta, x1, y1);
    rotate_point(q_x, q_y, q_theta, x2, y2);
    SDL_RenderDrawLine(gRenderer.get(), x1, y1, x2, y2); // top wing

    x1 = q_x - 80, y1 = q_y + 10;
    x2 = q_x + 80, y2 = q_y + 10;
    rotate_point(q_x, q_y, q_theta, x1, y1);
    rotate_point(q_x, q_y, q_theta, x2, y2);
    SDL_RenderDrawLine(gRenderer.get(), x1, y1, x2, y2); // bottom wing

    // Draw center body
    int bx1 = q_x - 80, by1 = q_y - 10;
    rotate_point(q_x, q_y, q_theta, bx1, by1);
    SDL_Rect bodyRect1 = { bx1, by1, 160, 20 };
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
    SDL_RenderFillRect(gRenderer.get(), &bodyRect1);

    int bx2 = q_x - 70, by2 = q_y - 60;
    rotate_point(q_x, q_y, q_theta, bx2, by2);
    SDL_Rect bodyRect2 = { bx2, by2, 10, 60 };
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_RenderFillRect(gRenderer.get(), &bodyRect2);

    int bx3 = q_x + 60, by3 = q_y - 60;
    rotate_point(q_x, q_y, q_theta, bx3, by3);
    SDL_Rect bodyRect3 = { bx3, by3, 10, 60 };
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_RenderFillRect(gRenderer.get(), &bodyRect3);

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
        rotate_point(q_x, q_y, q_theta, x, y);
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
        rotate_point(q_x, q_y, q_theta, x, y);
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
        rotate_point(q_x, q_y, q_theta, x, y);
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
        rotate_point(q_x, q_y, q_theta, x, y);
        right_lWing[i].x = x;
        right_lWing[i].y = y;
    }
    SDL_RenderDrawLines(gRenderer.get(), right_lWing, 6);
}