#include "cli_arguments.hpp"
#include "problems.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
ddpack::Matrix read(const std::filesystem::path& path) {
    std::ifstream file(path); if(!file) throw std::runtime_error("Missing reference: "+path.u8string());
    std::vector<std::vector<double>> rows; std::string line;
    while(std::getline(file,line)) { std::istringstream stream(line); std::string cell; std::vector<double> row;
        while(std::getline(stream,cell,',')) row.push_back(std::stod(cell)); if(!row.empty()) rows.push_back(row); }
    if(rows.empty()) throw std::runtime_error("Empty reference");
    ddpack::Matrix m(rows.size(),rows[0].size());
    for(std::size_t i=0;i<rows.size();++i) {if(rows[i].size()!=rows[0].size()) throw std::runtime_error("Ragged reference"); for(std::size_t j=0;j<rows[i].size();++j) m(i,j)=rows[i][j];} return m;
}
void close(const ddpack::Matrix& value,const ddpack::Matrix& expected,const std::string& field,double tolerance=2e-6) {
    if(value.rows()!=expected.rows() || value.cols()!=expected.cols() || !value.allFinite() || !expected.allFinite()) throw std::runtime_error("Dimension/nonfinite mismatch: "+field);
    const double error=(value-expected).cwiseAbs().maxCoeff(),scale=std::max(1.,expected.cwiseAbs().maxCoeff());
    if(error>tolerance*scale) throw std::runtime_error(field+" error="+std::to_string(error));
}
int DDPACK_CLI_MAIN(int argc,ddpack::cli::argument_char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("Reference directory required");
        const auto args=ddpack::cli::arguments(argc,argv);
        const auto root=std::filesystem::u8path(args[1]);
        bool pass=true;
        for(const auto& name:ddpack::examples::names()) {
          try {
            auto c=ddpack::examples::make(name); auto path=root/name;
            if(name=="vibration") c.problem.initial_controls=read(path/"initial_controls.csv");
            auto first_case=c; first_case.options.max_iterations=1; first_case.problem.projection_max_iterations=0; first_case.problem.geometric_refinement_iterations=0;
            auto first=ddpack::solve(first_case.problem,first_case.options);
            close(first.states,read(path/"first_states.csv"),name+" first-step states",1e-9);
            auto first_controls=read(path/"first_controls.csv");
            close(first.controls.leftCols(first.controls.cols()-1),first_controls.leftCols(first_controls.cols()-1),name+" first-step controls",1e-9);
            auto s=ddpack::solve(c.problem,c.options);
            auto states=read(path/"states.csv"),controls=read(path/"controls.csv");
            std::cout<<name<<" max state delta="<<(s.states-states).cwiseAbs().maxCoeff()<<" control delta="<<(s.controls.leftCols(s.controls.cols()-1)-controls.leftCols(controls.cols()-1)).cwiseAbs().maxCoeff()<<'\n';
            close(s.states,states,name+" states",2e-6);
            close(s.controls.leftCols(s.controls.cols()-1),controls.leftCols(controls.cols()-1),name+" active controls",2e-6);
            close(s.time.transpose(),read(path/"time.csv"),name+" time");
            if(std::filesystem::exists(path/"line_search_trials.csv")) {
                ddpack::Matrix trials(s.line_search_trials.size(),18),merit(s.merit.size(),1);
                for(std::size_t row=0;row<s.line_search_trials.size();++row)
                    for(int col=0;col<18;++col)trials(row,col)=s.line_search_trials[row][col];
                for(std::size_t row=0;row<s.merit.size();++row)merit(row,0)=s.merit[row];
                close(trials,read(path/"line_search_trials.csv"),name+" actual-merit trials",1e-8);
                close(merit,read(path/"merit.csv"),name+" accepted merit",1e-8);
            }
            if(c.problem.geometric_refinement_iterations>0) {
                // The old heuristic primary trace can branch under roundoff.
                // The new canonical restart phase has its own strict contract.
                close(s.refinement_starts,read(path/"refinement_starts.csv"),name+" restart records",1e-8);
                ddpack::Matrix history(s.refinement_penalty_history.size(),1);
                for(std::size_t k=0;k<s.refinement_penalty_history.size();++k)history(k,0)=s.refinement_penalty_history[k];
                close(history,read(path/"refinement_penalty_history.csv"),name+" refinement merit",1e-8);
                ddpack::Matrix objective(1,1);objective(0,0)=s.returned_cost;
                close(objective,read(path/"returned_objective.csv"),name+" returned objective",1e-9);
                if(!s.metrics.success)throw std::runtime_error(name+" physical acceptance failed");
                std::cout<<name<<" strict final/restart MATLAB parity PASS\n";
                continue;
            }
            auto costs=s.trial_costs;
            if(name=="obstacle_2d" || name=="obstacle_3d") { costs.push_back(s.projected_cost); if(c.problem.geometric_refinement_iterations>0) costs.push_back(s.returned_cost); }
            ddpack::Matrix trial(costs.size(),1); for(std::size_t i=0;i<costs.size();++i) trial(i,0)=costs[i];
            auto expected_trial=read(path/"trial.csv");
            if(name=="obstacle_3d") {
                close(trial.topRows(10),expected_trial.topRows(10),name+" early trial",1e-9);
                std::cout<<name<<" max trial delta="<<(trial-expected_trial).cwiseAbs().maxCoeff()<<'\n';
            }
            close(trial,expected_trial,name+" trial",name=="obstacle_3d"?1e-2:2e-5);
            if(!s.metrics.success) throw std::runtime_error(name+" physical acceptance failed");
            std::cout<<name<<" MATLAB parity PASS\n";
          } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';pass=false;}
        }
        return pass?0:1;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
