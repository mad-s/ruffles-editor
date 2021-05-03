#pragma once

#include <string>
#include "model/data_model.h"

namespace ruffles::model {

class Serializer
{

public:
	void serialize(const std::string& filename, DataModel& data_model);
	void deserialize(const std::string& filename, DataModel& data_model);

};
}