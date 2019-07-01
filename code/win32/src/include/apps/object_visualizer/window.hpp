#ifndef INCLUDED_WINDOW_HPP
#define INCLUDED_WINDOW_HPP

#include <SDL2/SDL.h>
#include <cstdint>

class i_window
{
public:
  virtual ~i_window()
  {
  }

  virtual int width() const = 0;
  virtual int height() const = 0;
  virtual void clear() = 0;
  virtual void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) = 0;
  virtual void update() = 0;
};

class window_2d: public i_window
{
public:
  window_2d(const char *title, int width, int height);
  ~window_2d() override;
  int width() const override;
  int height() const override;
  void clear() override;
  void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) override;
  void update() override;

private:
  SDL_Window *m_window;
  int m_width;
  int m_height;
};

class window_3d: public i_window
{
public:
  window_3d(const char *title, int width, int height);
  ~window_3d() override;
  int width() const override;
  int height() const override;
  void clear() override;
  void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) override;
  void update() override;

private:
  SDL_Window *m_window;
  SDL_GLContext m_ctx;
  int m_width;
  int m_height;
};


#endif  // INCLUDED_WINDOW_HPP
