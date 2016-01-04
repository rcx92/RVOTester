#include "MobaSimulator.h"

int main(){
	MobaSimulator *sim = new MobaSimulator(1000.0f, 1000.0f);
	while (true){
		sim->update(0.033);
		sim->show();
	}
}