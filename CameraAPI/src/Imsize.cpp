#include "Imsize.h"

uint32_t Imsize::width()
{
	return this->w;
}

uint32_t Imsize::height()
{
	return  this->h;
}

Imsize::Imsize(uint32_t width, uint32_t height)
{
	this->w = width;
	this->h = height;

}
