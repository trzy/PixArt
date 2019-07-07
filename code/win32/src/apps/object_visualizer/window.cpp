#include "apps/object_visualizer/window.hpp"
#include <GL/gl.h>
#include <stdexcept>

/*
 * window_2d:
 *
 * 2D window with bitmapped surface.
 */

window_2d::window_2d(const char *title, int width, int height)
  : m_window(SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN)),
    m_width(width),
    m_height(height)
{
  if (!m_window)
  {
    throw std::runtime_error("Failed to create window");
  }
}

window_2d::~window_2d()
{
  SDL_DestroyWindow(m_window);
}

SDL_Window *window_2d::window() const
{
  return m_window;
}

int window_2d::width() const
{
  return m_width;
}

int window_2d::height() const
{
  return m_height;
}

void window_2d::clear()
{
  SDL_Surface *surface = SDL_GetWindowSurface(m_window);
  SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0, 0, 0));
}

void window_2d::draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b)
{
  SDL_Surface *surface = SDL_GetWindowSurface(m_window);
  SDL_FillRect(surface, &rect, SDL_MapRGB(surface->format, r, g, b));
}

void window_2d::blit()
{
  SDL_UpdateWindowSurface(m_window);
}

/*
 * window_3d:
 *
 * OpenGL window.
 */

window_3d::window_3d(const char *title, int width, int height)
  : m_window(SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL)),
    m_width(width),
    m_height(height)
{
  if (!m_window)
  {
    throw std::runtime_error("Failed to create window");
  }

  m_ctx = SDL_GL_CreateContext(m_window);
  //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetSwapInterval(1);

  glClearColor(0, 0, 0, 1);
  glClearDepth(1.0);
  glViewport(0, 0, width, height);

  glFrontFace(GL_CW);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
}

window_3d::~window_3d()
{
  SDL_GL_DeleteContext(m_ctx);
  SDL_DestroyWindow(m_window);
}

SDL_Window *window_3d::window() const
{
  return m_window;
}

int window_3d::width() const
{
  return m_width;
}

int window_3d::height() const
{
  return m_height;
}

void window_3d::set_context()
{
  SDL_GL_MakeCurrent(m_window, m_ctx);
}

void window_3d::clear()
{
  set_context();
  glClear(GL_COLOR_BUFFER_BIT);
  glClear(GL_DEPTH_BUFFER_BIT);
}

void window_3d::draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b)
{
  set_context();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 1, 1, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glBegin(GL_QUADS);
  glColor3f(r / 255.0f, g / 255.0f, b / 255.0f);
  float xres = m_width;
  float yres = m_height;
  float left = rect.x / xres;
  float right = (rect.x + rect.w) / xres;
  float top = rect.y / yres;
  float bottom = (rect.y + rect.h) / yres;
  glVertex2f(left, top);
  glVertex2f(right, top);
  glVertex2f(right, bottom);
  glVertex2f(left, bottom);
  glEnd();
}

void window_3d::blit()
{
  set_context();
  SDL_GL_SwapWindow(m_window);
}