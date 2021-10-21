#pragma once

#include <string>
#include <vector>

class ToDo
{
private:
    std::vector<std::string> this_task;
public:    
    ToDo(/* args */);
    ~ToDo();

    size_t size() const;
    void addTask(const std::string& task);
    std::string getTask(size_t index) const;
};
