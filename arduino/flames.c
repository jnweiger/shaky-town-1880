#include <stdio.h>
#include <string.h>	// memcpy()
#include <stdlib.h>	// abs()
#include <unistd.h>	// usleep()

typedef struct pingpong {
  // the value oszillates between min and max. The movement direction is vec = dir * spe, where dir is either +1 or -1.
  // we split the sign away, so that we can keep the direction even after speed was temporarily 0.
  int val, min, max, spe, dir;
} pp;
 
struct flame {
  pp bright, bspeed, xpos, xspeed;
};

void pp_advance(struct pingpong *p)
{
  p->val += p->dir * p->spe;
  if (p->val > p->max)
    {
      p->dir = -p->dir;
      // reflect back, go the same distance under max, that we were above max.
      p->val -= 2* (p->val - p->max);
    }
  if (p->val < p->min)
    {
      p->dir = -p->dir;
      // reflect back, go the same distance over min, that we were under min.
      p->val += 2* (p->min - p->val);
    }
}

// pp_set_speed only sets the magnitude of vec, but does not change the direction.
// Make sure vec fits within the limits of min and max,
// this is good to check here, as pp_advance() assumes, that one advancement can never hit both, min and max.
void pp_set_speed(struct pingpong *p, int vec)
{
  // vec = abs(vec);
  int vmax = p->max - p->min;
  if (vec > vmax) vec = vmax;
  if (vec < -vmax) vec = -vmax;
  p->spe = vec;
}

void flame_advance(struct flame *f)
{
  // nodulate a flame. its brightness changes 
  pp_advance(&f->bright);
  pp_advance(&f->xpos);
  pp_advance(&f->xspeed);
  pp_set_speed(&f->xpos,   f->xspeed.val >> 3);	// divide by 8
  pp_set_speed(&f->bright, f->bspeed.val >> 3);	// divide by 8
}

#define N_FLAMES 4		// number of flames
#define N_ADV_PER_STEP 1	// number of advancements per loop
#define DECAC_PER_STEP 0.95	// decacy factor per advancement


struct flame f[N_FLAMES];

struct rgb {
  unsigned char r, g, b;
};

struct rgb fcolor[N_FLAMES*2] = 
{
  // cool
  { 133, 255, 155 },	// bright green
  { 188, 155, 255 }, 	// bright blue
  { 255, 255, 255 },	// white
  { 155, 225, 255 },	// bright cyan
  // warm
  { 255, 255,  77 },	// bright yellow
  { 255, 211,  55 }, 	// bright peaches
  { 255,  99,  55 },	// bright tomato
  { 222, 255, 111 }	// bright lemon
};

unsigned char decay[256];

#ifdef DEBUG
// char line[3*25+10];
#endif

void setup_flames()
{
  //                 val, min, max, spe, dir
  f[0].bright = (pp){  50, 0, 1024, 7, +1 };
  f[1].bright = (pp){ 350, 0, 1024, 9, -1 };
  f[2].bright = (pp){ 450, 0, 1024, 3, +1 };
  f[3].bright = (pp){ 555, 0, 1024, 1, +1 };

  f[0].bspeed = (pp){ 7*8, 5*8, 20*8,  7, -1 };
  f[1].bspeed = (pp){ 9*8, 7*8, 11*8, 10, +1 };
  f[2].bspeed = (pp){ 3*8, 0*8, 10*8,  3, +1 };
  f[3].bspeed = (pp){ 1*8, 0*8, 30*8, 22, +1 };

  f[0].xpos = (pp){ 222, 0, 16*3*25,  8, +1 };
  f[1].xpos = (pp){  50, 0, 16*3*25,  2, -1 };
  f[2].xpos = (pp){ 111, 0, 16*3*25, 15, +1 };
  f[3].xpos = (pp){ 278, 0, 16*3*25,  5, -1 };

  f[0].xspeed = (pp){ 8*8, 0*8, 13*8,  7, -1 };
  f[1].xspeed = (pp){ 2*8, 7*8, 23*8, 13, +1 };
  f[2].xspeed = (pp){15*8, 5*8, 17*8,  3, +1 };
  f[3].xspeed = (pp){ 5*8, 0*8,  7*8,  2, +1 };
#if N_FLAMES > 4
#error "main initializes only 4 flames"
#endif

  for (int i = 0; i < 256; i++)
     decay[i] = int(DECAC_PER_STEP * i);
}

void loop()
{
  int i;
  for (i = 0; i < 75; i++)
    {
      uint32_t color = getPixel(i);
      unsigned char r = (color >> 16) & 0xff;
      unsigned char g = (color >> 8) & 0xff;
      unsigned char b = (color >> 0) & 0xff;
      setPixel(i, (decay[r]<<16) | (decay[g]<<8) | (decay[b]<<0);
    }

  char temperature = 0;		// cold: 0  .. warm: N_FLAMES-1
  for (int j = 0; j < N_ADV_PER_STEP; j++)
    {
      for (i = 0; i < N_FLAMES; i++)
	{
	  int x = f[i].xpos.val >> 4;
	  
	  unsigned char r = ((unsigned int)f[i].bright.val * fcolor[i+temperature].r)>>10;
	  unsigned char g = ((unsigned int)f[i].bright.val * fcolor[i+temperature].g)>>10;
	  unsigned char b = ((unsigned int)f[i].bright.val * fcolor[i+temperature].b)>>10;
	  setPixel(i, (r<<16) | (g<<8) | (b<<0);
	  flame_advance(f+i);
	}
    }

  
#ifdef DEBUG
  // printf("0:%d\t x=%d+%d\n", f[0].bright.val, f[0].xpos.val, f[0].xpos.spe*f[0].xpos.dir);

#if 1
  // add the brighness values up to make an illumination profile for the [750] positions.
  // print out somehow...
  for (i = 0; i < sizeof(line)-1; i++) line[i] = '_';
  line[sizeof(line)-1] = '\0';

  for (i = 0; i < N_FLAMES; i++)
    {
      char b[10];
      int len = sprintf(b, "%d[%d]", i, f[i].bright.val);
      memcpy(line+(f[i].xpos.val>>4), b, len);
    }
  puts(line);	// this implicitly adds a linefeed
#endif
#if 0
  printf("0:%d\t x=%d+%d\t 1:%d\t x=%d+%d\t 2:%d\t x=%d+%d\t 3:%d\t x=%d+%d \n",
    f[0].bright.val, f[0].xpos.val, f[0].xpos.spe*f[0].xpos.dir, f[1].bright.val, f[1].xpos.val, f[1].xpos.spe*f[1].xpos.dir, 
    f[2].bright.val, f[2].xpos.val, f[2].xpos.spe*f[2].xpos.dir, f[3].bright.val, f[3].xpos.val, f[3].xpos.spe*f[3].xpos.dir);
#endif
  usleep(10000);
}

int main(int ac, char **av)
{
  setup();
  while (1) loop();
}
