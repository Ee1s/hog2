

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
	}
	else {
		InstallHandlers();
		RunHOGGUI(argc, argv);
	}
}
