#include "apps/object_visualizer/window.hpp"
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <stdexcept>

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

void window_2d::update()
{
  SDL_UpdateWindowSurface(m_window);
}
