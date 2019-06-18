#pragma once
#ifndef INCLUDED_PIXART_HPP
#define INCLUDED_PIXART_HPP

#include <cstdint>

struct PA_object;

uint32_t PA_get_frame_period_microseconds();
void PA_write(uint8_t bank, uint8_t reg, uint8_t data);
uint8_t PA_read(uint8_t bank, uint8_t reg);
void PA_read_report(uint8_t buffer[], int format);
void PA_read_report(PA_object objs[16], int format);
void PA_init();
void PA_deinit();

#endif  // INCLUDED_PIXART_HPP
