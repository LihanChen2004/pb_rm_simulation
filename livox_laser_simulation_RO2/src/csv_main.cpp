//
// Created by lfc on 2021/3/1.
//
#include "ros2_livox/csv_reader.hpp"

int main(int argc, char const* argv[]) {
    std::vector<std::vector<double>> datas;
    CsvReader::ReadCsvFile("/sros/avia.csv", datas);
    return 0;
}