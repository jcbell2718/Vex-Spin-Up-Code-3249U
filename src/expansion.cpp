#include "main.h"

Expansion::Expansion() :
    expansion(EXPANSION_PORT),
    expansion2(EXPANSION_PORT_2),
    center_expansion(CENTER_EXPANSION_PORT),
    expanded(false)
{
    expansion.set_value(true);
    expansion2.set_value(true);
    center_expansion.set_value(false);
}

void Expansion::expand() {
    expanded = true;
    expansion.set_value(false);
	expansion2.set_value(false);
    center_expansion.set_value(true);
}

void Expansion::reset() {
    expanded = false;
    expansion.set_value(true);
	expansion2.set_value(true);
    center_expansion.set_value(false);
}

void Expansion::toggle() {
    expanded = !expanded;
    expansion.set_value(!expanded);
	expansion2.set_value(!expanded);
    center_expansion.set_value(expanded);
}

Expansion expansion = Expansion();