#include "ToDo.hpp"

ToDo::ToDo(/* args */) {
}

ToDo::~ToDo() {
}

size_t ToDo::size() const {
    return this_task.size();
}

void ToDo::addTask(const std::string& task) {
    this_task.push_back(task);
}

std::string ToDo::getTask(size_t index) const {
    if (index < this_task.size())
        return this_task[index];
    else 
        return "";
}