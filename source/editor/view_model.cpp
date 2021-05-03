#include "editor/view_model.h"
#include "editor/elements/abstract_element.h"

namespace ruffles::editor
{

ViewModel::~ViewModel()
{
	for (AbstractElement* element : elements)
		delete element;
}

void ViewModel::add_element(AbstractElement* element)
{
	elements.push_back(element);
}

}
