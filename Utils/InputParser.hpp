//
//  InputParser.hpp
//  RobotHu
//
//  Created by Aleksandr Khorobrykh on 04/03/2017.
//  Copyright © 2017 AKTech. All rights reserved.
//

// Based on: http://stackoverflow.com/a/868894/2852163

#include <vector>
#include <string>

class InputParser {
    
public:
    
    InputParser (int argc, const char * argv[]);
    
    const std::string& getCmdOption(const std::string &option);
    bool cmdOptionExists(const std::string &option);

private:
    std::vector <std::string> tokens;
    std::string empty_string;
};
