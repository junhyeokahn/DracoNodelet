#include <stdio.h>
#include <iostream>
#include <data_manager/DataSave.hpp>

int main (int argc, char ** argv){
    DataSave data_save;
    data_save.start();

    int dummy;
    std::cin >> dummy;

    return 0;
}
