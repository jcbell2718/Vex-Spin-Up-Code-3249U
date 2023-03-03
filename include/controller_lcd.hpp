#pragma once
#ifndef _CONTROLLER_LCD_
#define  _CONTROLLER_LCD_

#include "main.h"

void display_on_both_controllers(std::string text, int line, int col=0);
void display_on(okapi::Controller controller, std::string text, int line, int col=0);
void clear_controllers();
void clear_controller(okapi::Controller controller);
void clear_line_on(okapi::Controller controller, int line);
void clear_line_on_both_controllers(int line);

void controller_lcd_fn();

#endif  //  _CONTROLLER_LCD_HPP_