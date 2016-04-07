#include "CamProperty.h"

CamProperty::CamProperty()
{
}

CamProperty::~CamProperty()
{
}

CamProperty::CamProperty(std::string name, boost::any value, std::string nameType)
{
	this->name = name;
	this->value = value;
	this->nameType = nameType;
}


std::string CamProperty::getName()
{
	return name;
}
