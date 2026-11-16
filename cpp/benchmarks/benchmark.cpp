#include "cli_arguments.hpp"
#include "problems.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

ddpack::Matrix initial_controls(const std::filesystem::path& file) {
    std::ifstream f(file); if(!f) throw std::runtime_error("Missing MATLAB initial-control fixture");
    std::vector<double> values; std::string line,cell; std::getline(f,line); std::istringstream row(line);
    while(std::getline(row,cell,',')) values.push_back(std::stod(cell));
    ddpack::Matrix m(1,values.size()); for(std::size_t i=0;i<values.size();++i)m(0,i)=values[i]; return m;
}
int DDPACK_CLI_MAIN(int argc,ddpack::cli::argument_char** argv) {
    try {
        const auto args=ddpack::cli::arguments(argc,argv);
        const std::filesystem::path root=argc>1?std::filesystem::u8path(args[1]):std::filesystem::path("cpp/tests/reference");
        const auto destination=argc>2?std::filesystem::u8path(args[2]):std::filesystem::path("artifacts/native_benchmark");
        if(std::filesystem::exists(destination)) throw std::runtime_error("Choose a new benchmark directory");
        std::filesystem::create_directories(destination);
        std::ofstream raw(destination/"timings.csv"); raw<<std::setprecision(17)<<"case,repeat,seconds,success,returned_cost\n";
        bool passed=true;
        for(const auto& name:ddpack::examples::names()) {
            auto c=ddpack::examples::make(name);
            if(name=="vibration") c.problem.initial_controls=initial_controls(root/name/"initial_controls.csv");
            for(int warm=0;warm<3;++warm) ddpack::solve(c.problem,c.options);
            std::vector<double> timings;
            for(int repeat=0;repeat<5;++repeat) {
                const auto begin=std::chrono::steady_clock::now(); auto s=ddpack::solve(c.problem,c.options);
                const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
                raw<<name<<','<<repeat+1<<','<<seconds<<','<<(s.metrics.success?1:0)<<','<<s.returned_cost<<'\n';
                passed=passed && s.metrics.success; timings.push_back(seconds);
                if(repeat==0) ddpack::write_artifacts(c.problem,c.options,s,(destination/name).u8string());
            }
            std::sort(timings.begin(),timings.end()); std::cout<<name<<" median seconds="<<timings[2]<<'\n';
        }
        return passed?0:1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
