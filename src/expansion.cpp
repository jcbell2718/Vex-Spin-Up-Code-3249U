#include "main.h"

Expansion::Expansion() :
    angry_birds(ANGRY_BIRDS_PORT),
    center_expansion(CENTER_EXPANSION_PORT),
    expanded(false)
{
    angry_birds.set_value(true);
    center_expansion.set_value(false);
}

void Expansion::expand() {
    expanded = true;
    angry_birds.set_value(false);
    center_expansion.set_value(true);
}

void Expansion::reset() {
    expanded = false;
    angry_birds.set_value(true);
    center_expansion.set_value(false);
}

void Expansion::toggle() {
    expanded = !expanded;
    angry_birds.set_value(!expanded);
    center_expansion.set_value(expanded);
}

Expansion expansion = Expansion();