#ifndef INCLUDED_WINDOW_HPP
#define INCLUDED_WINDOW_HPP

#include "pa_driver/pixart_object.hpp"
#include "pixart/settings.hpp"
#include <SDL2/SDL.h>
#include <cstdint>
#include <array>

class i_window
{
public:
  virtual ~i_window()
  {
  }

  virtual void init(const pixart::settings &settings) = 0;
  virtual void update(const std::array<PA_object, 16> &objs) = 0;
  virtual void blit() = 0;
  virtual SDL_Window *window() const = 0;
  virtual int width() const = 0;
  virtual int height() const = 0;

protected:
  virtual void clear() = 0;
  virtual void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) = 0;
};

class window_2d: public i_window
{
public:
  window_2d(const char *title, int width, int height);
  ~window_2d() override;

  virtual void init(const pixart::settings &settings) override
  {
  }

  virtual void update(const std::array<PA_object, 16> &objs) override
  {
  }

  void blit() override;

  SDL_Window *window() const override;
  int width() const override;
  int height() const override;

protected:
  void clear() override;
  void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) override;

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

  virtual void init(const pixart::settings &settings) override
  {
  }

  virtual void update(const std::array<PA_object, 16> &objs) override
  {
  }

  void blit() override;

  SDL_Window *window() const override;
  int width() const override;
  int height() const override;

protected:
  void set_context();
  void clear() override;
  void draw_rectangle(const SDL_Rect &rect, uint8_t r, uint8_t g, uint8_t b) override;

private:
  SDL_Window *m_window;
  SDL_GLContext m_ctx;
  int m_width;
  int m_height;
};

#endif  // INCLUDED_WINDOW_HPP
