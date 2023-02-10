#pragma once
#ifndef _CONTROLLER_LCD_
#define  _CONTROLLER_LCD_

#include "main.h"

void display_on_both_controllers(std::string text, int line, int col=0);
void clear_controllers();
void display_on(okapi::Controller controller, std::string text, int line, int col=0);

void controller_lcd_fn();

#endif  //  _CONTROLLER_LCD_HPP_