#pragma once
#include <string>
#include <vector>
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communication with Gnuplot

using namespace std;

class Plot{
public:
	Plot(uint numberOfPlotWindows=2){
		N = numberOfPlotWindows;
		figure.resize(N);
		for (uint n=0;n<N;n++) figure.at(n) = new Gnuplot;
	}
	std::vector<Gnuplot*> figure;
	uint N = 1;

	void array(double* array, uint N, std::string name, uint fig){
		if(fig<N) {
			std::vector<double> x, y;
			x.resize(N);
			y.resize(N);
			for (uint i = 0; i < N; i++) {
				x.at(i) = i;
				y.at(i) = array[i];
			}
			figure.at(fig)->set_style("lines").plot_xy(x, y, name);
		}
	}

	void array(uint* array, uint N, std::string name, uint fig){
		if(fig<N) {
			std::vector<uint> x, y;
			x.resize(N);
			y.resize(N);
			for (uint i = 0; i < N; i++) {
				x.at(i) = i;
				y.at(i) = array[i];
			}
			figure.at(fig)->set_style("lines").plot_xy(x, y, name);
		}
	}

	void array(float* array, uint N, std::string name, uint fig){
		if(fig<N) {
			std::vector<int> x, y;
			x.resize(N);
			y.resize(N);
			for (uint i = 0; i < N; i++) {
				x.at(i) = i;
				y.at(i) = array[i];
			}
			figure.at(fig)->set_style("lines").plot_xy(x, y, name);
		}
	}

	void array(vector<float> y, float dx, std::string name, uint fig){
		if(fig<N) {
			vector<float> x;
			x.resize(y.size());
			x[0]=0;
			for (uint i = 1; i < y.size(); i++) {
				x[i] = x[i-1]+dx;
			}
			figure.at(fig)->set_style("lines").plot_xy(x, y, name);
		}
	}

	void array(vector<float> x, vector<float> y, std::string name, uint fig){
		if(fig<N) {
			figure.at(fig)->set_style("lines").plot_xy(x, y, name);
		}
	}

	void clear(uint fig){
		if(fig<N) {
			figure.at(fig)->reset_all();
		}
	}

	void clearAll(){
		for (uint n=0;n<N;n++) figure.at(n)->reset_all();
	}
};