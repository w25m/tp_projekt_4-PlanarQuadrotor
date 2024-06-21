#pragma once

#include <Eigen/Dense>
#include <tuple>

#include "lqr.h"
#include <SDL.h>

class PlanarQuadrotor {
private:
    Eigen::VectorXf z = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf z_dot = Eigen::VectorXf::Zero(6);
    Eigen::VectorXf z_goal = Eigen::VectorXf::Zero(6);
    // m, I, r, g parameters
    Eigen::VectorXf params = Eigen::Vector4f(0.486, 0.00383, 0.25, 9.81);
    Eigen::Vector2f input = Eigen::Vector2f::Zero();

    SDL_AudioSpec wav_spec;
    Uint32 wav_length;
    Uint8* wav_buffer;
    SDL_AudioDeviceID device_id;

public:
    PlanarQuadrotor();
    PlanarQuadrotor(Eigen::VectorXf z);
    ~PlanarQuadrotor();
    void SetGoal(Eigen::VectorXf z_goal);
    Eigen::VectorXf GetState();
    Eigen::VectorXf GetControlState();
    Eigen::Vector2f GravityCompInput();
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> Linearize();
    void SetInput(Eigen::Vector2f input);
    void DoCalcTimeDerivatives(); // Shamelessly copied from Drake API
    void DoUpdateState(float dt);
    Eigen::VectorXf Update(Eigen::Vector2f& input, float dt);
    Eigen::VectorXf Update(float dt);
};
