#ifndef PARA_SERVER
#define PARA_SERVER

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <ios>
#include <atomic>
#include <memory>

namespace parameter_server{
class ParaeterSerer{
private:
    std::string file_path_;
    std::ifstream fin;
    std::atomic_flag para_lock_ = ATOMIC_FLAG_INIT; 
public:
    ParaeterSerer(const string file_path){
        file_path_ = file_path;
        std::cout << "[Parameter Server] Parameter file path: " << file_path_ << std::endl;
        fin.open(file_path_);
        if (!fin.good()){
            std::string msg("[Parameter Server] Parameter file not found! - ");
            msg.append(file_path_);
            throw std::runtime_error(msg);
        }
    }

    ~ParaeterSerer(){
        fin.close();
    }

    template <class Data>
    bool get_para(const std::string& name, Data& data)
    {
        std::string line;
        int para_value;
        while (para_lock_.test_and_set())
            ;
        fin.seekg(0, std::ios::beg);
        while (fin.good()){
            getline(fin, line);
            line = remove_str(line, ':');
            if (line[0] != '#'){
                std::string para_name;
                std::stringstream buffer;
                buffer.clear();
                buffer << line;
                buffer >> para_name;
            
                if (para_name.compare(name) == 0){
                    buffer >> data;
                    std::cout << "[Parameter Server] Get " << name << ": " << data << std::endl;
                    para_lock_.clear();
                    return true;
                }
                else{
                    buffer >> para_value;
                }
            }
        }
        // throw std::runtime_error(std::string("[Parameter Server] Unknown para:  ").append(name));
        std::cout << "\033[31m" << std::string("[Parameter Server] Unknown para:  ").append(name) << "\033[0m" << std::endl;
        para_lock_.clear();
        return false;
    }

private:
    std::string remove_str(const std::string& s, const char& skip){
        std::string temp(s);
        temp.erase(std::remove(temp.begin(), temp.end(), skip), temp.end());
        return temp;
    }
};



} // namespace parameter_server

#endif