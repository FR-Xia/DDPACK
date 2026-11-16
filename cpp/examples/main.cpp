#include "cli_arguments.hpp"
#include "problems.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <iomanip>
int DDPACK_CLI_MAIN(int argc,ddpack::cli::argument_char** argv) {
    try {
        const auto args=ddpack::cli::arguments(argc,argv);
        const std::string which=argc>1?args[1]:"all";
        const std::string directory=argc>2?args[2]:"artifacts/native";
        std::vector<std::string> cases=which=="all"?ddpack::examples::names():std::vector<std::string>{which};
        if(std::filesystem::exists(std::filesystem::u8path(directory))) throw std::runtime_error("Choose a new output directory");
        bool passed=true;
        for(const auto& name:cases) {
            auto c=ddpack::examples::make(name); auto begin=std::chrono::steady_clock::now();
            auto s=ddpack::solve(c.problem,c.options);
            double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            ddpack::write_artifacts(c.problem,c.options,s,directory+"/"+name);
            std::cout<<std::setprecision(9)<<name<<" terminal="<<s.metrics.terminal_error<<" euler="<<s.metrics.euler_defect<<" rk4="<<s.metrics.rk4_defect<<" clearance="<<s.metrics.minimum_clearance<<" cost="<<s.returned_cost<<" time="<<s.final_time<<" seconds="<<elapsed<<" "<<(s.metrics.success?"PASS":"FAIL")<<'\n';
            passed=passed && s.metrics.success;
        }
        return passed?0:1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 2;}
}
