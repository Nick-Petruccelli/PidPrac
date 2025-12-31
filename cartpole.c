#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>

/* ------------------------------
   Physical parameters
-------------------------------- */
typedef struct {
    double mc;   // cart mass
    double mp;   // pole mass
    double l;    // pole half-length
    double g;    // gravity
    double bc;   // cart damping (N·s/m)
    double bp;   // pole damping (N·m·s/rad)
} CartPoleParams;

typedef struct {
    double x;        // cart position (m)
    double x_dot;    // cart velocity (m/s)
    double theta;    // pole angle (rad, 0 = upright)
    double theta_dot;// pole angular velocity (rad/s)
} CartPoleState;

void cartpole_step(CartPoleState *s, const CartPoleParams *p, double force, double dt){
  double sin_t = sin(s->theta);
  double cos_t = cos(s->theta);

  double denom = p->mc + p->mp * sin_t * sin_t;

  double x_ddot =
    (force
    - p->bc * s->x_dot
    + p->mp * sin_t * (p->l * s->theta_dot * s->theta_dot + p->g * cos_t))
    / denom;

  double theta_ddot =
    (-force * cos_t
    - p->bp * s->theta_dot
    - p->mp * p->l * s->theta_dot * s->theta_dot * cos_t * sin_t
    - (p->mc + p->mp) * p->g * sin_t)
    / (p->l * denom);

  s->x_dot     += x_ddot * dt;
  s->x         += s->x_dot * dt;

  s->theta_dot += theta_ddot * dt;
  s->theta     += s->theta_dot * dt;
}
