//
//  InputParser.cpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 04/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

#include "InputParser.hpp"
#include <iostream>

InputParser::InputParser(int argc, const char * argv[]) {
    
    for (int i = 1; i < argc; ++i)
        this->tokens.push_back(std::string(argv[i]));
}

const std::string& InputParser::getCmdOption(const std::string &option) {
    
    std::vector<std::string>::const_iterator itr;
    itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
    if (itr != this->tokens.end() && ++itr != this->tokens.end()){
        return *itr;
    }
    return empty_string;
}

bool InputParser::cmdOptionExists(const std::string &option) {
    
    return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
}
