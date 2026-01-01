#include "cartpole.c"
#include <SDL2/SDL.h>
#include <SDL2/SDL_timer.h>
#include <math.h>
#include <stdint.h>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

#define CART_WIDTH 40
#define CART_HEIGHT 30
#define SCALE 200

#define KP_THETA 20.0
#define KD_THETA 15.0
#define KP_X 0.1
#define KI_X 0.0
#define KD_X 0.75
#define MAX_LEAN 0.05
#define MAX_X_I 1.0

double last_x_error = 0;
double x_error_sum = 0;
double last_theta_error = 0;
double theta_error_sum = 0;

void renderer_cartpole(SDL_Renderer *renderer, CartPoleState *state, CartPoleParams *params){
  SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
  SDL_RenderClear(renderer);
  int cart_x = (SCREEN_WIDTH / 2) + state->x * SCALE;
  int cart_y = SCREEN_HEIGHT / 2;

  SDL_Rect cart_rect = {
    cart_x - CART_WIDTH / 2,
    cart_y - CART_HEIGHT / 2,
    CART_WIDTH,
    CART_HEIGHT
  };
SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255); SDL_RenderFillRect(renderer, &cart_rect);

  int pole_x1 = cart_x;
  int pole_y1 = cart_y;
  int pole_x2 = cart_x + (params->l * sin(state->theta) * SCALE);
  int pole_y2 = cart_y + (params->l * cos(state->theta) * SCALE);

  SDL_SetRenderDrawColor(renderer, 250, 50, 50, 255);
  SDL_RenderDrawLine(renderer, pole_x1, pole_y1, pole_x2, pole_y2);

  SDL_RenderPresent(renderer);
}

double pid_controler(CartPoleState *state, double dt){
  double x_error = -state->x;
  double x_dirivative = -state->x_dot;
  last_x_error = x_error;
  x_error_sum += x_error * dt;
  if(x_error_sum > MAX_X_I)
    x_error_sum = MAX_X_I;
  if(x_error_sum < -MAX_X_I)
    x_error_sum = -MAX_X_I;
  double theta_target = M_PI + (KP_X * -x_error + KI_X * -x_error_sum - KD_X * x_dirivative);
  theta_target = theta_target > M_PI + MAX_LEAN ? M_PI + MAX_LEAN : theta_target < M_PI - MAX_LEAN ? M_PI - MAX_LEAN : theta_target;
  double theta_error = theta_target - state->theta;
  double theta_dirivative = -state->theta_dot;
  last_theta_error = theta_error;
  return KP_THETA * theta_error + KD_THETA * theta_dirivative;
}

int main(int argc, char *argv[])
{
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window *window = SDL_CreateWindow("Cartploe PID SIM",SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);


  CartPoleParams params = {
      .mc = 1.0,
      .mp = 0.1,
      .l  = 0.5,
      .g  = 9.81,
      .bc = 0.1,   // cart friction
      .bp = 0.05   // pole air friction
  };

  CartPoleState state = {
      .x = 1.0,
      .x_dot = 0.0,
      .theta = M_PI - 0.01,   // small initial angle (rad)
      .theta_dot = 0.0
  };

  double dt = 0.001;     // 1 kHz control loop
  double freq = SDL_GetPerformanceFrequency();
  double last_time_step = SDL_GetPerformanceCounter();

  int running = 1;
  while(running){
    uint64_t now = SDL_GetPerformanceCounter();
    double time_since_step = (now - last_time_step) / freq;
    if(time_since_step >= dt){
      double force = pid_controler(&state, dt);
      force = fmax(fmin(force, 10.0), -10.0);
      cartpole_step(&state, &params, force, dt);
      printf("Theta Error: %lf\n", M_PI - state.theta);
      printf("X Error: %lf\n", -state.x);
      last_time_step = SDL_GetPerformanceCounter();
    }

    SDL_Event e;
    while(SDL_PollEvent(&e)){
      if(e.type == SDL_QUIT){
        running = 0;
        break;
      }
    }

    renderer_cartpole(renderer, &state, &params);

  }

  return EXIT_SUCCESS;
}
