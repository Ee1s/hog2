

#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <vector>
#include "GUICode.h"
#include "Timer.h"






int main(int argc, char* argv[])
{
	if (argc > 1 && strcmp(argv[1], "-stp") == 0)
	{
		//lambdaPriorityQueue<xyLoc> fward(2.0, 0);
		//lambdaPriorityQueue<xyLoc> bward(2.0, 0);
		//WMM<xyLoc, tDirection, MapEnvironment, lambdaPriorityQueue<xyLoc>> mm(2.0, 2.0, fward, bward);

		//lambdaPriorityQueue<xyLoc> wfward(2.0, 1);
		//lambdaPriorityQueue<xyLoc> wbward(2.0, 1);
		//WMM<xyLoc, tDirection, MapEnvironment, lambdaPriorityQueue<xyLoc>> wmmcompare(2.0, 2.0, wfward, wbward);

		////TemplateAStar<xyLoc, tDirection, MapEnvironment> wmmcompare;
		//bool mmSearchRunning = false;
		//bool wmmcompareSearchRunning = false;

		//wmmcompare.InitializeSearch(me, start, goal, me, me, path);
		//mm.InitializeSearch(me, start, goal, me, me, path);
	}
	else {
		InstallHandlers();
		RunHOGGUI(argc, argv);
	}
}
