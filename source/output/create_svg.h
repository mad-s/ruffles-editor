#pragma once

#include <tinyxml2.h>
#include "common/common.h"
#include "ruffle/ruffle.h"

namespace ruffles {

unique_ptr<tinyxml2::XMLDocument> create_svg(Ruffle &ruffle);

}
